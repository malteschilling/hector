#include <stdlib.h>
#include <functional>

#include <boost/asio.hpp>
#include <boost/assign.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>



#include <BfbMessage.hpp>
#include "BfbProtocolIds.hpp"
#include "SerialInterface.hpp"
#include <TcpServer.hpp>
#include "NotificationTimer.hpp"

unsigned long portNum;



////////////////////////////////////////// Main
int main (int argc, char **argv)
{	
	// Declare and parse the program options specified via command line
	// Declare the supported options.
	
	boost::program_options::options_description desc("Command line options");
	desc.add_options()
	("help", "produce help message")
	("port", boost::program_options::value<unsigned short>()->default_value(50002), "define the TCP port the server will listen on")
	("print", boost::program_options::value<bool>()->default_value(false), "print every message that is received via the serial or the TCP interface")
	("maxPayloadPrintout", boost::program_options::value<signed long int>()->default_value(-1), "Sets the maximum number of payload bytes that will be printed. This might be helpful if the output of the geometry xml should not be printed completely.")
	("resend", boost::program_options::value<unsigned int>()->default_value(3), "set the number of transmission attempts the server will undertake in order to get a reply for a message for which a reply is expected.")
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

	portNum=vm["port"].as<unsigned short>();
	
	// Create the interfaces to the serial and the network interfaces. 
	TcpServer  TcpInter(portNum);
	SerialInterface  SerialInter;
	NotificationTimer Timer(TcpInter.GetSendMessageHandle());
	
	SerialInter.SetNumOfTransmissionAttempts(vm["resend"].as<unsigned int>());
	
	TcpInter.NotifyOfNewConnection(boost::bind(&NotificationTimer::ResetTimer, &Timer, _1));
	
	// Give the two interfaces a handle to the respectively other one.
	SerialInter.RouteIncomingMessagesTo(TcpInter.GetSendMessageHandle());
	
	boost::function<void (boost::shared_ptr<const BfbMessage>)> ProcessIncomingTcpMessages=[&](boost::shared_ptr<const BfbMessage> message)->void{
		//BfbFunctions::printMessage(message);
		if(message->GetProtocol()==BfbProtocolIds::SIMSERV_1_PROT){
			auto reply=boost::make_shared<BfbMessage>(message->GetRawData());
			reply->SetDestination(message->GetSource());
			reply->SetSource(message->GetDestination());
			reply->SetCommand(message->GetCommand()+1);
			switch (message->GetCommand()){
				case 30:
					reply->SetPayload(boost::assign::list_of<unsigned char>(TcpInter.GetTcpConnectionBroadcastState(message->GetSource())));
					break;
				case 32:
				{
					bool state=false;
					auto payload=message->GetPayload();
					for(auto it=payload.begin(); it!=payload.end(); it++){
						if(*it>0){
							state=true;
							break;
						};
					};
					TcpInter.SetTcpConnectionBroadcastState(message->GetSource(), state);
					break;
				};
				default:
					Timer.ProcessMessage(message);
					return;
					break;
			};
			if(message->GetBusAllocation()){
				TcpInter.SendMessage(reply);
			};
		}else{
			SerialInter.SendMessage(message);
		};
	};
	
	TcpInter.RouteIncomingMessagesTo(ProcessIncomingTcpMessages);
	
	// If the print option was specified, also add the print function to the message signals.
	
	if(vm["print"].as<bool>()){
		std::cout<<"All received messages will be printed."<<std::endl;
		signed long int maxPayloadPrintout=vm["maxPayloadPrintout"].as<signed long int>();
		
		if(maxPayloadPrintout>0){
			std::cout<<"The printout of the payload is limited to "<<std::dec<<maxPayloadPrintout<<" bytes."<<std::endl;
		}
		
		std::function<void (boost::shared_ptr<const BfbMessage>)> serialPrint=[maxPayloadPrintout](boost::shared_ptr<const BfbMessage> message){
			BfbFunctions::printMessage(message, "The following message was received via the serial interface.", maxPayloadPrintout);
		};
		SerialInter.RouteIncomingMessagesTo(serialPrint);
		
		std::function<void (boost::shared_ptr<const BfbMessage>)> tcpPrint=[maxPayloadPrintout](boost::shared_ptr<const BfbMessage> message){
			BfbFunctions::printMessage(message, "The following message was received via the TCP interface.", maxPayloadPrintout);
		};
		TcpInter.RouteIncomingMessagesTo(tcpPrint);
	};
	
	// Now the communication should be set up. In order to stop itself, the program waits for a keyboard input.
	
	std::cout<<"The server is now ready to receive messages."<< std::endl;
	std::cout<<"It listens on port "<< portNum << "."<< std::endl;
	auto clientList=SerialInter.GetConnectedClients();
	std::cout<<"The following "<< clientList.size() << " clients are ready for communication:"<< std::endl;
	for(auto it=clientList.begin();it!=clientList.end();it++){
		std::cout<<std::dec<<std::right<<std::setw(4)<<int(*it)<<"( "<<std::hex<< std::showbase<<int(*it)<<" ), "<<std::endl;;
	};
	auto serialPortNames=SerialInter.GetSerialPortNames();
	std::cout<<"The following serial ports are connected to bus masters:"<< std::endl;
	for(auto it=serialPortNames.begin();it!=serialPortNames.end();it++){
		std::cout<<*it<<std::endl;;
	};
	
	while(true){
		std::cout<<"If you want to stop the server, enter 'x' and hit return."<<std::endl;
		std::string keyboardInput;
		getline(std::cin, keyboardInput);
		if(keyboardInput=="x"){
			break;
		};
	};
	
	return 0;
}
