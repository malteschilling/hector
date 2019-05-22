// STL includes
#include <stdlib.h>
#include <iostream>
#include <iterator>
#include <queue>
#include <string>
#include <vector>

// Boost includes
#include <boost/asio.hpp>
#include <boost/asio/basic_serial_port.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/assign.hpp>
#include <boost/bind.hpp>
#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>
#include <boost/pointer_cast.hpp>
#include <boost/regex.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/weak_ptr.hpp>

// Own header files
#include <BfbMessage.hpp>
#include "SerialInterface.hpp"

static unsigned char line0DeactivationStartId=0x90;
static unsigned char line1DeactivationStartId=line0DeactivationStartId+0x10;

/** \brief This function searches for all serial ports that match the signature of a BioFlex bus master. The names of the available ports are returned as strings. 
 * 	It was written by Thierry Hoinville.
 */
std::vector<std::string> parseSerialPorts(){
	using namespace std;
	using namespace boost::filesystem;
	path p ("/dev"); 
	
	#if defined __APPLE__
	const boost::regex my_filter( "ttyusbmodem.*" );
	#elif defined __linux__
	//const boost::regex my_filter( "ttyA.*" );
	const boost::regex my_filter( "ttyA.*" );
	#else
	#error "unknown platform"
	#endif
	
	
	std::vector<std::string> serialPortNames;
	
	try {
		if (exists(p)) {   // does p actually exist?
			if (is_regular_file(p)){        // is p a regular file?
				//cout << p << " size is " << file_size(p) << '\n';
			
			}else if (is_directory(p)) {     // is p a directory?
				//cout << p << " is a directory containing:\n";
				
				for( boost::filesystem::directory_iterator i( p ); i != directory_iterator(); ++i ) {
					boost::smatch what;
					if( !boost::regex_match( i->path().filename().generic_string(), what, my_filter ) ) continue;
						serialPortNames.push_back(std::string("/dev/").append(i->path().filename().generic_string()));
					// Open this serial port and check it
					// ...
				}
			}else{
				//cout << p << " exists, but is neither a regular file nor a directory\n";
			};
		} else{
			//cout << p << " does not exist\n";
		};
	} catch (const filesystem_error& ex) {
		cout << ex.what() << '\n';
	}
	return serialPortNames;
}
/** These are the different states a serial connection may assume. */
enum serialConnectionInitialisationState_t {	WaitingForBusMasterIdentificationReply, 
						WaitingForBusClientIdentificationReplyOnLine0, 
						WaitingForBusClientIdentificationReplyOnLine1, 
						InitialisationComplete};


class ExtendedBfbMessage: public BfbMessage{
	public:
		ExtendedBfbMessage(const BfbMessage message):
			BfbMessage(message){
		};
		mutable unsigned char 		NumOfTransmissions=0;
		mutable boost::system_time 	TimeOfLastTransmission=boost::posix_time::microsec_clock::local_time();
		mutable std::function<void (boost::shared_ptr<const BfbMessage>)> CallBackFunction=nullptr;
};


class SerialConnection
{
	public:
		/** \brief The constructor 
		 * \param ioService The connection handler for asynchronous communication
		 * \param serialPortName The name of the serial port this serial client is supposed to use for communication
		 * \param incomingMessageSignal This is used for the signaling of received messages. Modules that should be informed about a received message must be connected to this signal.
		 * \param registerClientFunction A function that will be called if a client devise was found (an actuator, a sensor, in general a devise that understands the BioFlex protocol).
		 */ 
		SerialConnection(boost::shared_ptr<boost::asio::io_service> ioService, std::string serialPortName, boost::function<void (boost::shared_ptr<const BfbMessage>)> incomingMessageSignal);
		~SerialConnection();
		/** \brief Method responsible for sending messages via the serial port that was assigned to the respective instance of this class.
		 * \param message The message that should be send.
		 */
		void SendMessage(boost::shared_ptr<const BfbMessage> Message);
		
		/** \brief This method blocks until the initialisation of the serial port and the identification of the connected clients is completed. */
		void WaitUntilInitialised();
		
		/** \brief The method returns a sorted list of clients connected via the instance. 
		 * \return A sorted list of clients that can be reached by an instance. 
		 */
		std::list<unsigned char> GetConnectedClients();
		
		void SetNumOfTransmissionAttempts(unsigned int numOfTransmissionAttempts);
		
		std::string GetSerialPortName();
		
		void CloseConnection();
		
	private:
		/** \brief Method that must be called in order to start the receiving automatism.
		 * This is done in the constructor once. It is then called whenever one message has been received completely and a new message should be received.
		 */ 
		void TryToReceiveMessages();
		
		/**  \brief This method should be called by the asynchronous IO-Handler whenever the first eight bytes of a message have been received. Dependend on whether the received bytes represent a complete short message or only the first part of a long message, the message-receipt will be signaled and the message receival automatism will be restarted or - for the latter case - the asynchronous communication framework will be configured in order to receive the number of bytes that is missing to construct a long message. */
		void HandleReceivedShortMessageOrLongHeader(const boost::system::error_code& error,
			size_t bytes_transferred);
		
		/** \brief The "HandleReceivedShortMessageOrLongHeader" configures the asynchronous interface such that the missing number of bytes will be received in case only the header of a long package was received. After they were received, this function must be called. It signals the receipt of a message to the connected receivers and restarts the receival automatism. */
		void HandleReceivedLongMessage(const boost::system::error_code& error,
			size_t bytes_transferred);
		
		/** \brief This is not one of the handling methods that are called directly from the IOService object upon asynchronous receipt of a certain number of bytes. It must be called from one of the asynchronous methods if a message has been received completely. This method then deals with the message. */
		void HandleReceivedMessage(boost::shared_ptr<BfbMessage> incomingMessage);
		
		/** \brief The "SendMessage" method does not send the messages directly. It merely pushes them into a queue. Afterwards, it will call this method that is responsible for configuring the asynchronous interface such that after the topmost message has been sent, the corresponding handler ("HandleSentMessage") will be called that calls this function again to prepare the next message for sending. In this way, all messages that are in the queue will be sent one after another.*/
		void SendNextMessage();
		
		/** \brief This method will be called whenever a message has been sent. It will then call the "SendNextMessage" method in order to prepare the next message for sending.*/
		void HandleSentMessage(boost::shared_ptr<const BfbMessage> message, const boost::system::error_code& error);
		
		bool IsActive=true; /*!< This variable represents the status of the serial connection. If it is true, messages can be send and received. If it is false, the connection has been closed (probably because no bus master was connected or because the bus master was unplugged during operation) and therefore no communication is possible. */ 
		
		boost::shared_ptr<boost::asio::io_service> IoService; /*!< Asynchronous communication handler used by the instance to connect to the socket and to call the handler methods. */

		std::string SerialPortName; /*!< Name of the serial port. */
		boost::asio::serial_port SerialPort; /*!< Serial port handle. */
		serialConnectionInitialisationState_t InitialisationState; /*!< Status variable that is used during the initialisation in order to reflect the different stages. */
		
		std::function<void (boost::shared_ptr<const BfbMessage>)> IncomingMessageCallbackFunction; /*!> In this variable, the reference to the signaling function is saved. The corresponding signla will be called every time a message was received. */
		
		std::queue<boost::shared_ptr<const BfbMessage>> MessagesToBeSend; /*!< Queue in which all messages are teporarily saved before they are sent.*/
		bool IsSendPending; /*!< Status variable signaling whether a message is waiting to be sent completely.*/
		bool BusMasterDetected=false; /*!< Status variable that signals whether a bus master has been found on this serial port so far. */
		boost::shared_ptr<boost::recursive_mutex> ExclusiveAccessMutex; /*!< This mutex is used to make sure only one thread accesses the send methods at the same time. */
		boost::shared_ptr<boost::mutex> InitialisationMutex; /*!< This mutex is used to make sure the initialisation is finished before a member method is called. */
		
		static const unsigned int MaxMessageLength=256; /*!< This variable defines the maximum size a message can have that is supposed to be received using this module.*/
		std::vector<unsigned char> IncomingData; /*!< This variable holds the received bytes until a complete message has been received and it can be converted into an appropriate object. It should only be used in the handler methods! If the bytes are modified inbetween, the message will be undecipherable! */
		std::vector<unsigned char> OutgoingData; /*!< This variable holds the to-be-send bytes. Again: Do not modify the contents except for within the corresponding handler methods! */
		
		boost::asio::deadline_timer InitialisationTimer; /*!< Timer that is used during the initialisation of the instance. It sets an upper boundary for the time a certain step of the initialisation may last. */
		
		/** \brief The method handles the expiration of the initialisation timer. The effect of the expiration depends on the state of the instance (see definition). */
		void HandleExpiredInitialisationTimer(const boost::system::error_code& error);
				
		std::list<unsigned char> ClientsOnLine0; /*!< In this list, all client IDs are saved that are found on line 0 of the bus master. This information is only neccessary during initialisation. */
		std::list<unsigned char> ClientsOnLine1; /*!< In this list, all client IDs are saved that are found on line 1 of the bus master. This information is only neccessary during initialisation. */
		
		/** \brief The method handles the expiration of the resend timer. This typically means that a reply was not received although one was expected. */
		void HandleExpiredResendTimer(const boost::system::error_code& error);
		std::list<boost::shared_ptr<const ExtendedBfbMessage>> UnansweredRequests; /*!< List storing all sent messages for which no answer has been received so far. If an answer is received, the corresponding request will be deleted from this list.*/
		boost::asio::deadline_timer ResendTimer; /*!< Timer instance is used to make sure that an answer was received within a certain time. If this time is exceeded, the request will be resent.*/
		bool IsResendTimerActive=false; /*!< Flag that is used to tell whether an asynchronous wait has been configured for the ResendTimer.*/
		boost::posix_time::time_duration TimeToWaitForResponse=boost::posix_time::milliseconds(2); /*!> The maximum time between a request and the corresponding reply before the request is resent.*/
		
		unsigned int NumOfTransmissionAttempts=3;
	
};


SerialInterface::SerialInterface():
		Work(*IoService),
		IoServiceThread(boost::bind(&boost::asio::io_service::run,IoService)),
		SerialConnections(std::vector<boost::shared_ptr<SerialConnection>>()),
		Clients(std::map<unsigned char, boost::shared_ptr<SerialConnection>>()){
	
	std::vector<std::string> serialPortNames=parseSerialPorts(); // Get the names of the serial ports.
	std::vector<boost::shared_ptr<SerialConnection>> tempSerialConnections=std::vector<boost::shared_ptr<SerialConnection>>();
	std::function<void(boost::shared_ptr<const BfbMessage> message)> tempFunction= boost::bind(&SerialInterface::ForwardIncomingMessage, this,_1);
	for(unsigned int i=0;i<serialPortNames.size();i++){
			auto temp=boost::make_shared<SerialConnection>(IoService, serialPortNames[i], tempFunction);
			temp->SetNumOfTransmissionAttempts(NumOfTransmissionAttempts);
			tempSerialConnections.push_back(temp); // Create a new instance of the serial connection class for every serial port that is available. 
	};
	//std::cout<<"Started all "<< int(SerialConnections.size()) << " serial connections."<<std::endl;
	// Now, all the serial connection instances are trying to identify clients. In order to send messages to them, their IDs must be connected to the corresponding serial connection. Therefore, one must wait until the serial connection has been initialised before the client IDs can be read.
	for(unsigned int i=0;i<tempSerialConnections.size();i++){
		tempSerialConnections[i]->WaitUntilInitialised();
		auto tempList=tempSerialConnections[i]->GetConnectedClients();
		if(tempList.size()>0){
			SerialConnections.push_back(tempSerialConnections[i]);
			for(auto it=tempList.begin();it!=tempList.end();it++){
				Clients[*it]=tempSerialConnections[i];
			};
		};
		//std::cout<<"Serial connection Nr. " << int(i) << " is ready for operation."<<std::endl;
	};
	
}

SerialInterface::~SerialInterface(){
	IoService->stop();
	IoServiceThread.join();
};


void SerialInterface::SetNumOfTransmissionAttempts(unsigned int numOfTransmissionAttempts){
	for(auto it=SerialConnections.begin(); it!=SerialConnections.end(); it++){
		(*it)->SetNumOfTransmissionAttempts(numOfTransmissionAttempts);
	};
}


std::list<unsigned char> SerialInterface::GetConnectedClients(){
	std::list<unsigned char> tempList;
	for(auto it=Clients.begin();it!=Clients.end();it++){
		tempList.push_back(it->first);
	};
	tempList.sort();
	return tempList;
}

std::list< std::string > SerialInterface::GetSerialPortNames(){
	std::list< std::string > tempNames;
	for(auto it=SerialConnections.begin();it!=SerialConnections.end();it++){
		tempNames.push_back((*it)->GetSerialPortName());
	};
	return tempNames;
	
}


boost::function<void (boost::shared_ptr<const BfbMessage>)> SerialInterface::GetSendMessageHandle(){
	return [&, this](boost::shared_ptr<const BfbMessage> outputMessage)->void{
		if(this){
			this->SendMessage(outputMessage);
		};
	};
};


void SerialInterface::SendMessage(boost::shared_ptr<const BfbMessage> message){
	auto it=Clients.find(message->GetDestination());
	if(it!=Clients.end()){
		it->second->SendMessage(message);
	};
};

void SerialInterface::ForwardIncomingMessage(boost::shared_ptr< const BfbMessage > message){
	for(auto it=InputMessagesRouteList.begin(); it!=InputMessagesRouteList.end(); it++){
		(*it)(message);
	};
}

void SerialInterface::RouteIncomingMessagesTo(boost::function<void (boost::shared_ptr<const BfbMessage>)> forwardFunction){
	InputMessagesRouteList.push_back(forwardFunction);
	return;
};


/** \brief This method creates a message instance that triggers an identity reply from the client it is sent to. */
BfbMessage createIdentificationRequestMessageForId(unsigned char destinationId){
	BfbMessage identificationRequest = BfbMessage();
	identificationRequest.SetDestination(destinationId);
	identificationRequest.SetSource(2);
	identificationRequest.SetProtocol(1);
	identificationRequest.SetCommand(0);
	identificationRequest.SetBusAllocationFlag(true);
	return identificationRequest;
};

////////////////////////////////////////////////////////////////////////////////

SerialConnection::SerialConnection(boost::shared_ptr<boost::asio::io_service> ioService, const std::string serialPortName, boost::function<void (boost::shared_ptr<const BfbMessage>)> incomingMessageSignal):
			IoService(ioService),
			SerialPortName(serialPortName),
			SerialPort(*ioService, serialPortName),
			InitialisationState(WaitingForBusMasterIdentificationReply),
			IncomingMessageCallbackFunction(incomingMessageSignal),
			MessagesToBeSend(std::queue<boost::shared_ptr<const BfbMessage>>()),
			IsSendPending(false),
			ExclusiveAccessMutex(new boost::recursive_mutex),
			InitialisationMutex(new boost::mutex),
			IncomingData(std::vector<unsigned char>(0)),
			OutgoingData(std::vector<unsigned char>(0)),
			InitialisationTimer(*ioService),
			ClientsOnLine0(std::list<unsigned char>()),
			ClientsOnLine1(std::list<unsigned char>()),
			UnansweredRequests(std::list<boost::shared_ptr<const ExtendedBfbMessage>>()),
			ResendTimer(*ioService){

	InitialisationMutex->lock();
	IncomingData.reserve(MaxMessageLength);
	OutgoingData.reserve(MaxMessageLength);
	auto rawData=createIdentificationRequestMessageForId(1).GetRawData();
	for(int i=0;i<3;i++){
		boost::asio::write(SerialPort, boost::asio::buffer(rawData, rawData.size()));
	};
	
	
	InitialisationTimer.expires_from_now(boost::posix_time::seconds(1));	
	InitialisationTimer.async_wait(boost::bind(&SerialConnection::HandleExpiredInitialisationTimer, this, boost::asio::placeholders::error));
	TryToReceiveMessages();
}
SerialConnection::~SerialConnection(){
	CloseConnection();
};

void SerialConnection::CloseConnection(){
	IsActive=false;
	try{
		SerialPort.cancel();  // will cause read_callback to raise an error
	}catch(...){}
	try{
		SerialPort.close();  
	}catch(...){}
}


std::string SerialConnection::GetSerialPortName(){
	return SerialPortName;
}


void SerialConnection::SetNumOfTransmissionAttempts(unsigned int numOfTransmissionAttempts){
	NumOfTransmissionAttempts=numOfTransmissionAttempts;
}


void SerialConnection::WaitUntilInitialised(){
	boost::lock_guard<boost::mutex> lock(*InitialisationMutex); // The mutex gets unlocked as soon as the initialization has been completed.
	return;
}

std::list<unsigned char> SerialConnection::GetConnectedClients(){
	std::list<unsigned char> tempList=ClientsOnLine0;
	tempList.merge(ClientsOnLine1);	
	tempList.sort();
	tempList.unique();
	return tempList;
}
/** \brief The method creates a message that configures one of the two lines of the bus master to forward messages of a certain address range.
 * \param isLine1 Controls whether line 0 or line 1 should be configured. 
 * \param startId Defines the address range that will be forwarded by the specified line. The range is always startId...startId+0x0F.
 * \return message that must be sent to the bus master for configuration.
 */
BfbMessage createBusMasterConfigurationMessage(bool isLine1, unsigned char startId){
	BfbMessage busMasterConfiguration = BfbMessage();
	busMasterConfiguration.SetDestination(1);
	busMasterConfiguration.SetSource(2);
	busMasterConfiguration.SetProtocol(1);
	if(isLine1){
		busMasterConfiguration.SetCommand(0x80);
	}else{
		busMasterConfiguration.SetCommand(0x82);
	};
	busMasterConfiguration.SetBusAllocationFlag(true);
	busMasterConfiguration.SetPayload(boost::assign::list_of(240)(startId));// First element is the mask that enables the range startID...startId+0x10
	return busMasterConfiguration;
};

/** \brief This method handles the initalisation routine of a serial connection instance.
 * During the initialisation, different tasks must be concluded:
 *  - Find out if a bus master connected?
 *  - Find out which clients are connected to line 0 of the bus master?
 *  - Find out which clients are connected to line 1 of the bus master?
 *  - Configure the bus master such that all clients will be available. 
 * Since the tasks use asynchronous communication, it is convenient to always use a timer object to set an upper boundary for the time a task may take up. If a task can be completed before this time is over, the timer can be interrupted. 
 */
void SerialConnection::HandleExpiredInitialisationTimer(const boost::system::error_code& error)
{
	//std::cout<<"HandleExpiredInitialisationTimer"<<std::endl;
	std::vector<unsigned char>  tempRawData(2,0);
	std::vector<unsigned char>  busMasterConfigurationRawData;	
	
	switch(InitialisationState){
		case WaitingForBusMasterIdentificationReply: // In the constructor method, an identification request has been sent to the bus master. Afterwards, the timer was set. If the timer ran out without having received a reply from the bus master, it must be assumed that no bus master is connected. 
			//std::cout<<"WaitingForBusMasterIdentificationReply"<<std::endl;
			if (!error || !BusMasterDetected){ // Identification was not received and therefore this timeout was not canceled. This means that no Bioflex Bus Master is connected on this serial port.
				CloseConnection(); 
				InitialisationMutex->unlock(); // The initalisation has been completed prematurely.
				return;
			}
			// If we came to this point, a bus master is connected to this serial port. Now, the connected clients must be identified. First on line 0.

		// Attention: Do not put a "break" here! The following code can be reused by the second initalisation stage. 	
		case WaitingForBusClientIdentificationReplyOnLine0:
			//std::cout<<"WaitingForBusClientIdentificationReplyOnLine0"<<std::endl;
			{ //Case
			// Change the status 
			if(InitialisationState==WaitingForBusMasterIdentificationReply){
				InitialisationState=WaitingForBusClientIdentificationReplyOnLine0;
			}else if(InitialisationState==WaitingForBusClientIdentificationReplyOnLine0){
				InitialisationState=WaitingForBusClientIdentificationReplyOnLine1;
			}
			//Configure the bus master such that the "other line" is basically not sending any messages
			if(InitialisationState==WaitingForBusClientIdentificationReplyOnLine0){
				busMasterConfigurationRawData=createBusMasterConfigurationMessage(1,line1DeactivationStartId).GetRawData();
			}else if(InitialisationState==WaitingForBusClientIdentificationReplyOnLine1){
				busMasterConfigurationRawData=createBusMasterConfigurationMessage(0,line0DeactivationStartId).GetRawData();
			};
			boost::asio::write(SerialPort, boost::asio::buffer(busMasterConfigurationRawData, busMasterConfigurationRawData.size()));
				

			
			//Checking Clients on the currently interesting line
			BfbMessage identificationRequest = createIdentificationRequestMessageForId(0x00);
			unsigned char confStartId;
			for(unsigned int repetition=0;repetition<10;repetition++){//Send each identification request multiple times in order to increase the chance one of them will come through
				confStartId=0x00;
				for(unsigned char destId=0x10;destId<line0DeactivationStartId;destId++){
					if((confStartId+0x0f)<destId){
						//Configure the bus master such that the currently interesting line is able to send messages in the desired ID range
						confStartId+=0x10;
						if(InitialisationState==WaitingForBusClientIdentificationReplyOnLine0){
							busMasterConfigurationRawData=createBusMasterConfigurationMessage(0,confStartId).GetRawData();
						}else if(InitialisationState==WaitingForBusClientIdentificationReplyOnLine1){
							busMasterConfigurationRawData=createBusMasterConfigurationMessage(1,confStartId).GetRawData();
						};
						boost::asio::write(SerialPort, boost::asio::buffer(busMasterConfigurationRawData, busMasterConfigurationRawData.size()));
					};
					identificationRequest.SetDestination(destId);
					tempRawData=identificationRequest.GetRawData();
					boost::asio::write(SerialPort, boost::asio::buffer(tempRawData, tempRawData.size()));
				};
			};
			InitialisationTimer.expires_from_now(boost::posix_time::milliseconds(500));
			InitialisationTimer.async_wait(boost::bind(&SerialConnection::HandleExpiredInitialisationTimer, this, boost::asio::placeholders::error));
			};//End of case
		break;
		case WaitingForBusClientIdentificationReplyOnLine1: // Both lines have been tested. Basically, all initalisation steps are completed. However, some housekeeping must be carried out. 
			//std::cout<<"WaitingForBusClientIdentificationReplyOnLine1"<<std::endl;
			//Sort the client lists
			ClientsOnLine0.sort();
			ClientsOnLine0.unique();
			ClientsOnLine1.sort();
			ClientsOnLine1.unique();
			
			if(ClientsOnLine0.size()>0 && ClientsOnLine1.size()>0 && !(ClientsOnLine0.back()<ClientsOnLine1.front() || ClientsOnLine1.back()<ClientsOnLine0.front())){
				std::cout<<"The client ID ranges for the two lines of the bus master connected to port "<< SerialPortName <<"  are intersecting. The bus master will be shut down."<<std::endl;
				CloseConnection();
				InitialisationMutex->unlock();
				return;
			}
			
			//Configure the bus master in order to forward the messages to the correct lines
			if(ClientsOnLine0.size()>0){
				busMasterConfigurationRawData=createBusMasterConfigurationMessage(0,(ClientsOnLine0.front()/16)*16).GetRawData();
			}else{
				busMasterConfigurationRawData=createBusMasterConfigurationMessage(0,line0DeactivationStartId).GetRawData();
			}
			boost::asio::write(SerialPort, boost::asio::buffer(busMasterConfigurationRawData, busMasterConfigurationRawData.size()));
			
			if(ClientsOnLine1.size()>0){
				busMasterConfigurationRawData=createBusMasterConfigurationMessage(1,(ClientsOnLine1.front()/16)*16).GetRawData();
			}else{
				busMasterConfigurationRawData=createBusMasterConfigurationMessage(1,line1DeactivationStartId).GetRawData();
			}
			boost::asio::write(SerialPort, boost::asio::buffer(busMasterConfigurationRawData, busMasterConfigurationRawData.size()));
			
			InitialisationState=InitialisationComplete; // Now, the initalisation is complete.
			InitialisationMutex->unlock();
		break;
		case InitialisationComplete: // This case should not happen because the timer should not be set after the prior step, however, since the compiler constantly complains about the missing case, here it is.
			break;
		}
    return;
}

void SerialConnection::TryToReceiveMessages(){
	IncomingData.resize(8);
	boost::asio::async_read(SerialPort, boost::asio::buffer(IncomingData, 8),
		boost::bind(&SerialConnection::HandleReceivedShortMessageOrLongHeader, this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
}


void SerialConnection::HandleExpiredResendTimer(const boost::system::error_code& error)
{
	if(!error){
		boost::lock_guard<boost::recursive_mutex> lock(*ExclusiveAccessMutex); 
		while(UnansweredRequests.size()>=1){
			boost::posix_time::time_duration deltaT= (boost::posix_time::microsec_clock::local_time()-UnansweredRequests.front()->TimeOfLastTransmission);
			if( deltaT.total_microseconds() > (TimeToWaitForResponse.total_microseconds()-10) ){
				SendMessage(UnansweredRequests.front());
				UnansweredRequests.pop_front();
			}else{
				break;
			};
		};
		if(!UnansweredRequests.empty()){
			boost::posix_time::time_duration deltaT= (boost::posix_time::microsec_clock::local_time()-UnansweredRequests.front()->TimeOfLastTransmission);
			ResendTimer.expires_from_now( TimeToWaitForResponse-deltaT );
			ResendTimer.async_wait(boost::bind(&SerialConnection::HandleExpiredResendTimer, this, boost::asio::placeholders::error));
			IsResendTimerActive=true;
		}else{
			IsResendTimerActive=false;
		};
	};
	IsResendTimerActive=false;
}

void SerialConnection::HandleReceivedMessage(boost::shared_ptr<BfbMessage> incomingMessage){
	switch(InitialisationState){
		case InitialisationComplete:
		{
			boost::lock_guard<boost::recursive_mutex> lock(*ExclusiveAccessMutex); 
			if(incomingMessage->GetSource()>=0x10 && incomingMessage->GetSource()<line0DeactivationStartId){
				IncomingMessageCallbackFunction(incomingMessage);	
				for(auto it=UnansweredRequests.begin();it!=UnansweredRequests.end();it++){
					if((*it)->GetDestination()==incomingMessage->GetSource() && (*it)->GetProtocol()==incomingMessage->GetProtocol() && (*it)->GetCommand()+1==incomingMessage->GetCommand()){
						UnansweredRequests.erase(it);
						break;
					};
				}
			};
		};
		break;
		case WaitingForBusMasterIdentificationReply:
			if(incomingMessage->GetSource()==1){
				BusMasterDetected=true;
				InitialisationTimer.cancel();
			};
		break;
		case WaitingForBusClientIdentificationReplyOnLine0:
			if(incomingMessage->GetSource()!=1){
				ClientsOnLine0.push_back(incomingMessage->GetSource());
			};
		break;
		case WaitingForBusClientIdentificationReplyOnLine1:
			if(incomingMessage->GetSource()!=1){
				ClientsOnLine1.push_back(incomingMessage->GetSource());
			};
		break;
	};
}


void SerialConnection::HandleReceivedShortMessageOrLongHeader(const boost::system::error_code& error, size_t bytesTransferred){
	//std::cout<<"Received data:"<<std::endl;
	if (error){
		CloseConnection();
		return;
	}
	
	if(BfbFunctions::isValidShortPacket(IncomingData)){//Short packet
		boost::shared_ptr<BfbMessage> incomingMessage(new BfbMessage(IncomingData));
		HandleReceivedMessage(incomingMessage);
		TryToReceiveMessages();
	}else if(BfbFunctions::isValidLongPacketHeader(IncomingData)){//Long packet
		unsigned int missingNumOfBytes=BfbFunctions::numOfMissingBytes(IncomingData);

		IncomingData.resize(missingNumOfBytes+bytesTransferred,0);
		boost::asio::async_read(SerialPort, boost::asio::buffer(IncomingData.data()+bytesTransferred, missingNumOfBytes),
			boost::bind(&SerialConnection::HandleReceivedLongMessage, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
		return;
	}else{
		//resynchronize
	};

}

void SerialConnection::HandleReceivedLongMessage(const boost::system::error_code& error,
    size_t bytes_transferred){
	if (error){
		CloseConnection();
		return;
	}

	if( BfbFunctions::isValidLongPacket(IncomingData) ){//Long packet
		boost::shared_ptr<BfbMessage> incomingMessage(new BfbMessage(IncomingData));
		HandleReceivedMessage(incomingMessage);
	}else{
		//resynchronize
	};
	TryToReceiveMessages();

}

void SerialConnection::SendMessage(boost::shared_ptr<const BfbMessage> message){
	boost::lock_guard<boost::recursive_mutex> lock(*ExclusiveAccessMutex); 
	if(IsActive){
		MessagesToBeSend.push(message);
		if(!IsSendPending){
			SendNextMessage();
		};
	};
};

void SerialConnection::SendNextMessage(){//Don't call this function without having locked the ConnectionMutex before. 
	boost::lock_guard<boost::recursive_mutex> lock(*ExclusiveAccessMutex); 
	boost::shared_ptr<const BfbMessage> tempMessage;
	while(!MessagesToBeSend.empty()){
		tempMessage=MessagesToBeSend.front();
		MessagesToBeSend.pop();
		if(tempMessage->GetPayload().size()<=151){
			break;
		}
	}
	OutgoingData=tempMessage->GetRawData();
	IsSendPending=true;
	boost::asio::async_write(SerialPort, boost::asio::buffer(OutgoingData.data(), OutgoingData.size()),
				boost::bind(&SerialConnection::HandleSentMessage, this, tempMessage, 
					boost::asio::placeholders::error));  
};

void SerialConnection::HandleSentMessage(boost::shared_ptr<const BfbMessage> message, const boost::system::error_code& error){
	boost::lock_guard<boost::recursive_mutex> lock(*ExclusiveAccessMutex); 
	if (error){
		CloseConnection();
		return;
	}
	boost::shared_ptr<const ExtendedBfbMessage> extMessage= boost::dynamic_pointer_cast<const ExtendedBfbMessage>(message);
	if(InitialisationState==InitialisationComplete && message->GetBusAllocation() && message->GetProtocol()!=0x09){
		if(extMessage==nullptr ){
			extMessage=boost::make_shared<ExtendedBfbMessage>(*message);
		};
		extMessage->NumOfTransmissions+=1;
		extMessage->TimeOfLastTransmission=boost::posix_time::microsec_clock::local_time();
		if(extMessage->NumOfTransmissions<NumOfTransmissionAttempts){
			//if(extMessage->NumberOfTransmissions>=3){
			//	std::cout<<"Sent message "<<std::dec<<int(extMessage->NumberOfTransmissions)<< " times"<<std::endl;
			//};
			UnansweredRequests.push_back(extMessage);
			//std::cout<<"UnansweredRequests has a size of "<<std::dec<< int(UnansweredRequests.size()) << std::endl;
			if(UnansweredRequests.size()==1){
				boost::posix_time::time_duration deltaT= (boost::posix_time::microsec_clock::local_time()-UnansweredRequests.front()->TimeOfLastTransmission);
				ResendTimer.expires_from_now( TimeToWaitForResponse-deltaT );
				ResendTimer.async_wait(boost::bind(&SerialConnection::HandleExpiredResendTimer, this, boost::asio::placeholders::error));
				IsResendTimerActive=true;
			};
		};
	}
	if(MessagesToBeSend.empty()){
		IsSendPending=false;
	}else{
		SendNextMessage();
	};
}
