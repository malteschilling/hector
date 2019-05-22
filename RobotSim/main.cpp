// STL includes
#include <stdlib.h>
#include <iostream>
#include <mutex>
#include <string.h>
#include <typeinfo>
#include <queue>

// Boost includes
#include <boost/asio.hpp>
#include <boost/assign/ptr_list_of.hpp>
#include <boost/assign.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/program_options.hpp>
#include <boost/numeric/ublas/matrix.hpp>

// Own header files
#include <BfbMessage.hpp>
#include "BfbMessageProcessor.hpp"
#include <TcpServer.hpp>
#include "DataTypes.hpp"
#include "GeometricPrimitives.hpp"
#include "GeometryXmlParser.hpp"
#include "HelperFunctions.hpp"
#include "OdeDrawstuff.hpp"
#include "Bodies.hpp"
#include "ThreadSafeQueue.hpp"
#include "Universe.hpp"


/** 
 * \brief "Uni" is the environment in which the simulation will run. 
 * The universe defines the vector of gravity, the "softness" and the damping that will be used for collisions etc, 
 *  It is basically possible to use multiple universes with a single instance of ODE, however, it has not yet been tested due to an absence of reasons to do so.
 */
boost::shared_ptr<Universe> universe;
/**
 * \brief "TcpInter" is the module that is responsible for the communication (via TCP). 
 * It handles all the incoming messages and singnals the receival of a message to all registered functions.
 * Therefore, if a function is supposed to deal with incoming messages, this function must be registered via the "RouteIncomingMessagesTo" member method. 
 * If messages should be sent via the TCP-interface, just get a handle to the 'send messages' method via the "GetSendMessageHandle" method. 
 * As an alternative, the member method "SendMessage" can be used directly. 
 */
boost::shared_ptr<TcpServer> tcpServer;

/** \brief This is the queue in which the incoming messages will be temporarily buffered before they are processed.
 * This enables the use of the SimulationLoop function in combination with the asynchronously working TCP-interface. 
 * The ThreadSafeQueue is responsible for the handling of incoming messages. When a message is received by the TCP-interface, 
 * the TCP-interface distributes it to all registered functions. One way to handle the incoming message would be to directly register the 
 * function that will interpret the message and read/modify some part of the universe (for example the velocity of a drive). In this case, 
 * every message would be sent to this function and the function can then create the response. This concept, however, is not compatible with 
 * the way the drawstuff library works. This library expects a function it can call for every screen update. Due to the asynchronous operation
 * of the TCP-interface, this would require additional measures to prevent simultanious access to variables. The easiest way to implement 
 * this concept would be to read the messages from the buffer and interpret them. 
 * In order to use the existing TCP library that was also used for the BioFlex server (in order to connect the real drives to a TCP connection),
 * a mixture of both approaches was chosen. Therefore, a queue was introduced in which the messages will be buffered. So, whenever a message is 
 * received, the TCP-interface will push the message into this queue via the 'Push' function that must be registered in the TCP-interface.
 * In the drawstuff update function ('SimulationLoop' function), the received messages can then be retrieved from the queue via the 
 * 'Pop' function.
 */
ThreadSafeQueue<boost::shared_ptr<const BfbMessage>> incomingMessages;

/** \brief This variable specifies whether a window should be opended for visualization. 
 * If no visualisation is active, the simulation will run faster, but you won't see what's happening...
 */
bool visualizationActive=false;
double continuousTimeFactor=0;

/** This variable specifies the maximum frequency with which the screen should be updated. 
 *The actual frequency, however, depends on different factors. See the comment for the "SimulationLoop" function. */
double visualisationFrequency=24;

/** \brief This function handles the steps that must be processed for each loop of the simulation. 
 * Actually, the function itself does not loop the simulation. Therefore, it must be looped in some other function. 
 * First of all, incoming messages must be handled. If a message specifies that the physics engine should simulate a certain period, this will block the screen updates. 
 * Also, as long as there are messages in the input buffer, they will be handled first. Therefor, the screen will be updated only if no messages are to be handled,
 * As a consequence, the visualisation will run at a maximum frequency specified by "visualisationFrequency". (The clouds in the visualization are not affected by this concept. 
 * They will keep on moving altough time is practically frozen. )
 * \param pause This parameter is a requirement from the drawstuff library. As it isn't documented at all, it is assumed that it is used as a switch to pause the simulating part of the SimulationLoop. 
 */
void SimulationLoop(const int pause){
	static boost::posix_time::ptime lastVisualizationTime=boost::get_system_time(); //pretend that the visualization must be updated as soon as possible when this is evaluated the first time (consider the "static" keyword).
	boost::system_time breakTime=boost::posix_time::pos_infin;
	if(visualizationActive || continuousTimeFactor>0){
		breakTime=boost::get_system_time()+boost::posix_time::microseconds(pow(10,6)/visualisationFrequency);
	};
	
	while(!pause){ // run this loop until there's no message left in the queue (the loop terminates via a break statement)
		boost::shared_ptr<const BfbMessage> incomingMessage;	
		try{
			incomingMessage=incomingMessages.PopUntil(breakTime);//lastVisualizationTime+boost::posix_time::microseconds(pow(10,6)/visualisationFrequency)); // Try to retrieve a message from the 
		}catch(const std::range_error& err){
			break;
		};
		//BfbFunctions::printMessage(incomingMessage);
		
		boost::shared_ptr<const BfbMessage> outgoingMessage=BfbMessageProcessor::ProcessMessage(universe,incomingMessage); // The received message will be handled by the message processor.
		/**
		* The BioFlex protocol that is used for the communication defines that a client (for example one of the drives of a robot) can reply to a message only if the bus master
		* has allocated the bus to the client. Therefore, in the message that the bus master sent to the client, the bus allocation flag must be set. If this flag has not been set,
		* the client cannot reply to a message. Also, the client cannot send a message on his own. It is only allowed to reply to a message (if it has bus allocation). 
		*/
		if(incomingMessage->GetBusAllocation() && outgoingMessage){ // Get the state of the bus allocation flag and test whether the outgoingMessage variable holds a valid pointer to a message. 
			tcpServer->SendMessage(outgoingMessage); // Send the reply.
		};
	};
	if(continuousTimeFactor>0 && !pause){
		universe->Simulate( continuousTimeFactor*( double((boost::get_system_time()-lastVisualizationTime).total_microseconds())/pow(10,6)) );
	};
	if(visualizationActive){ // If the visualisation is active, ...
		lastVisualizationTime=boost::get_system_time();
		universe->Draw();
	};

};

/** \brief This is another function needed for the initialization of the visualization. 
 * In this function, the view point and other visualization specific parameters can can be specified. 
 */
static void start()
{
    static float xyz[3] = {4.0f,-4.0f,1.7600f};
    static float hpr[3] = {140.000f,-17.0000f,0.0000f};
    dsSetViewpoint (xyz,hpr);
}


////////////////////////////////////////// Main
int main (int argc, char **argv)
{	
	/** Declare and parse the program options specified via command line. */
		// Declare the supported options.
		boost::program_options::options_description desc("Command line options");
		desc.add_options()//This command continues over the next few lines.
		("help", "produce help message")
		("port", boost::program_options::value<unsigned short>()->default_value(50002), "Defines the TCP port the simulation will listen on.")
		("visualize", boost::program_options::value<bool>()->default_value(true),"Shows/hides the 3D visualisation.")
		("windowWidth", boost::program_options::value<unsigned int>()->default_value(1200),"Sets the width of the window for the 3D visualization.")
		("windowHeight", boost::program_options::value<unsigned int>()->default_value(0), "Sets the height of the window for the 3D visualization. If not set (=0), a ration of 0.6250 relative to the width will be used.")
		("timeFactor", boost::program_options::value<double>()->default_value(0),"Runs the simulation continuously (without the need to explicitly send commands to iterate through time) with the specified factor relative to real time. 10: fast forward; 1: real time;  0.1: slow motion; 0: only 'simulate for some time'-commands are considered.")
		("print", boost::program_options::value<bool>()->default_value(false), "Prints every message that is received/sent via the TCP interface.")
		("maxPayloadPrintout", boost::program_options::value<signed long int>()->default_value(-1), "Sets the maximum number of payload bytes that will be printed. This might be helpful if the output of the geometry xml should not be printed completely.")
		("defaultRobot", boost::program_options::value<bool>()->default_value(false), "Loads the default robot (defined in the file 'DefaultRobot.xml').")
		;
		
		//Parse the options
		boost::program_options::variables_map vm;
		boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
		boost::program_options::notify(vm);    
		
		// Modify the program behaviour accordingly
		if (vm.count("help")) { // Show the help text and stop the program.
			std::cout << desc << std::endl;
			return 1;
		}
		unsigned long portNum=vm["port"].as<unsigned short>();
		visualizationActive=vm["visualize"].as<bool>();
		continuousTimeFactor=vm["timeFactor"].as<double>();

	/** Just for information purposes, the specified compiler options for the used ODE version are printed. */
	std::cout<<"ODE was built with the following options:"<<std::endl;
	std::cout<<dGetConfiguration()<<std::endl;
	
	/** The universe is created. As it turns out, this takes less than a second (<<7 days). */
	universe=boost::make_shared<Universe>();

	/** The TCP-interface is initialized. A free port must be specified. 
	 *If it is not explicitly specified via the command line, the default value [see above in the boost::program_options part] will be used. */
	tcpServer=boost::shared_ptr<TcpServer>(new TcpServer(portNum));
	
	/** Register the queue input method to the TCP-interface output method. */
	tcpServer->RouteIncomingMessagesTo(incomingMessages.GetPushHandle());
	
	// If the print option was specified, also add the print function to the message signals.
	if(vm["print"].as<bool>()){
		void (*print)(boost::shared_ptr<const BfbMessage>, std::string, signed long int) = &BfbFunctions::printMessage;
		tcpServer->RouteIncomingMessagesTo(boost::bind((print), _1, std::string("Incoming message via TCP:"), vm["maxPayloadPrintout"].as<signed long int>()));
		tcpServer->RouteOutgoingMessagesTo(boost::bind((print), _1, std::string("Outgoing message via TCP:"), vm["maxPayloadPrintout"].as<signed long int>()));
	};
	
	/** After the initialization, the TCP-connection can be used for communication.*/
	std::cout<<"The server is now ready to receive messages."<< std::endl;
	std::cout<<"It listens on port "<< portNum << "."<< std::endl;
	
	/** Run the xml-parser on the hector xml. This will create a robot in the simulation whose drives are ready for communication. */
	universe=GeometryXmlParser::Process("DefaultUniverse.xml",universe);
	if(vm["defaultRobot"].as<bool>()){
		universe=GeometryXmlParser::Process("DefaultRobot.xml",universe);
	};

	/** At last, the visualization is started. 
	 * If drawstuff is used, it calls the "simulation loop" function endlessly. 
	 * If it is not used, the "simulation loop" function must be called in an endless loop.
	 */
	if(visualizationActive){
		dsFunctions fn;
		fn.version = DS_VERSION;
		fn.start = &start;
		fn.step = &SimulationLoop;
		fn.stop = 0;
		fn.command = 0;
		fn.path_to_textures = "./textures";
		dsSetSphereQuality(2);
		dsSetCapsuleQuality(4);
		unsigned int windowWidth=vm["windowWidth"].as<unsigned int>();
		unsigned windowHeight=vm["windowHeight"].as<unsigned int>();
		if(windowHeight==0){
			windowHeight=0.6250*windowWidth;
		};
		dsSimulationLoop (argc,argv,windowWidth,windowHeight,&fn);
	}else{
		while(true){
			SimulationLoop(0);
		}
	}
	return 0;
}
