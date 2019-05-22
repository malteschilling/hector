// STL includes
#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <tuple>

// Boost includes
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>
#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>
#include <boost/weak_ptr.hpp>

// Own header files
#include "BfbClient.hpp"
#include <BfbMessage.hpp>
#include "CommunicationInterface.hpp"
#include "CommunicationXmlParser.hpp"

CommunicationInterface::CommunicationInterface(std::string tcpIp, std::string portNum):
		IoService(boost::make_shared<boost::asio::io_service>()),
		Work(boost::make_shared<boost::asio::io_service::work>(*IoService)),
		IoServiceThread(boost::bind(&boost::asio::io_service::run,IoService)),
		UnansweredMessagesMutex(new boost::mutex ),
		ResendTimer(*IoService){
	boost::asio::ip::tcp::resolver resolver(*IoService);
	boost::asio::ip::tcp::resolver::query query(tcpIp, portNum);
	boost::shared_ptr<boost::asio::ip::tcp::socket> socket=boost::make_shared<boost::asio::ip::tcp::socket>(*IoService);
	try{
		boost::asio::connect(*socket, resolver.resolve(query));
	}catch(...){
		throw std::invalid_argument("Can't connect to the simulator - the simulator has to be running.");
	}
	std::function<void(boost::shared_ptr< const BfbMessage >)> tempFunction=boost::bind(&CommunicationInterface::HandleIncomingMessage, this, _1);
	/*[this](boost::shared_ptr< const BfbMessage > message){
		this->HandleIncomingMessage(message);
	};*/
	TcpConn=boost::make_shared<TcpConnection>(IoService, socket, 0, tempFunction);
};

CommunicationInterface::~CommunicationInterface(){
	IoService->stop();
	IoServiceThread.join();
}

void CommunicationInterface::ParseProtocolXmls(std::vector< std::string > xmls){
	for(auto it=xmls.begin(); it!=xmls.end(); it++){
		CommunicationInterface::ParseProtocolXml(*it);
	};
}


void CommunicationInterface::ParseProtocolXml(std::string xml){
	CommunicationXmlParser::AttributeMap tempMap=CommunicationXmlParser::Process(xml);
	std::set<unsigned char> tempProtIdSet;
	for(auto protIt=tempMap.begin();protIt!=tempMap.end();protIt++){
		if(protIt->second.size()!=0){
			if( tempProtIdSet.count(protIt->second.begin()->GetProtocolId())==0 && DefaultAttributes.count(protIt->first)==0){
				tempProtIdSet.insert(protIt->second.begin()->GetProtocolId());
				for(auto attrIt=protIt->second.begin(); attrIt!=protIt->second.end(); attrIt++){
					attrIt->SetIterationNumberHandle([this]()->unsigned long int{
													return GetIterationNumber();
													}
									);
					attrIt->SetSendMessageHandle([this](boost::shared_ptr< const BfbMessage > message, 
									    std::function<void (boost::shared_ptr<const BfbMessage>)> replyHandler)->void{
										SendMessage(message, replyHandler);
										}
									);
				};
				DefaultAttributes.insert(tempMap.begin(), tempMap.end());
			}else{
				throw std::invalid_argument("A protocol with the id '"+ boost::lexical_cast<std::string>(int(protIt->second.begin()->GetProtocolId())) + "' or the name '" + protIt->first + "'is defined multiple times. Protocol-IDs must be unique!");
			};
		};
	};
}

void CommunicationInterface::NotifyOfNextIteration(){
	IterationNumber++;
}

unsigned long int CommunicationInterface::GetIterationNumber(){
	return IterationNumber;
}

void CommunicationInterface::HandleIncomingMessage(boost::shared_ptr< const BfbMessage > message){
	std::list< boost::shared_ptr<const ExtendedBfbMessage> > tempMessageList;
	{
		boost::lock_guard<boost::mutex> lock(*UnansweredMessagesMutex);
		unsigned char messageSource=message->GetSource();
		unsigned char messageProtocol=message->GetProtocol();
		signed short int messageCommand=message->GetCommand();
		auto oldMessage=UnansweredMessages.begin();
		while(oldMessage!=UnansweredMessages.end()){
			if((*oldMessage)->GetDestination()==messageSource && (*oldMessage)->GetProtocol()==messageProtocol && int((*oldMessage)->GetCommand())+1==messageCommand){
				tempMessageList.push_back((*oldMessage));
				oldMessage=UnansweredMessages.erase(oldMessage);
			}else{
				oldMessage++;
			};
		};
	};
	if(tempMessageList.size()==0){
		UnassignableMessages.push_back(message);
	}else{
		for(auto oldMessage=tempMessageList.begin(); oldMessage!=tempMessageList.end(); oldMessage++){
			(*oldMessage)->CallBackFunction(message);
		};
	};
};

void CommunicationInterface::SendMessage(boost::shared_ptr< const BfbMessage > message, std::function<void (boost::shared_ptr<const BfbMessage>)> replyHandler){
	//std::cout<<"Sending this message:"<<std::endl;
	//BfbFunctions::printMessage(message);
	auto extMessage= boost::dynamic_pointer_cast<const ExtendedBfbMessage>(message);
	if(!extMessage && replyHandler && message->GetBusAllocation()){
		extMessage=boost::make_shared<ExtendedBfbMessage>(*message);
		extMessage->CallBackFunction=replyHandler;
	};
	if(extMessage){
		extMessage->TimeOfLastTransmission=boost::posix_time::microsec_clock::local_time();
		extMessage->NumOfTransmissions+=1;
		
		boost::lock_guard<boost::mutex> lock(*UnansweredMessagesMutex);
		UnansweredMessages.push_back(extMessage);
		
		if(UnansweredMessages.size()==1){
			boost::posix_time::time_duration deltaT= (boost::posix_time::microsec_clock::local_time()-UnansweredMessages.front()->TimeOfLastTransmission);
			ResendTimer.expires_from_now( TimeToWaitForResponse-deltaT );
			ResendTimer.async_wait(boost::bind(&CommunicationInterface::HandleExpiredResendTimer, this, boost::asio::placeholders::error));
			IsResendTimerActive=true;
		};
	};
	TcpConn->SendMessage(message);
}


boost::shared_ptr<BfbClient > CommunicationInterface::CreateBfbClient(short unsigned int bioFlexBusId, std::vector< std::string > protocols){
	static const std::function<void (boost::shared_ptr<const BfbMessage>, std::function<void (boost::shared_ptr<const BfbMessage>)>)> sendMessageHandle =[this](boost::shared_ptr<const BfbMessage> message, std::function<void (boost::shared_ptr<const BfbMessage>)> handleReply)->void{
		return SendMessage(message, handleReply);
	};
	static const std::function<unsigned long int ()> getIterationNumberHandle =[this]()->unsigned long int{
		return GetIterationNumber();
	};
	
	std::vector<Attribute> attributeVec;
	for(auto protIt=protocols.begin(); protIt!=protocols.end(); protIt++){
		CommunicationXmlParser::AttributeList tempList;
		try{
			tempList=DefaultAttributes.at(*protIt);
		}catch(const std::out_of_range& error){
			throw std::out_of_range("You tried to create a client that is able to use the protocol '"+*protIt+"'. This protocol, however, is not available. The reason might be either that it has not been parsed previously using the ParseProtocolXml(s) method of the CommunicationInterface class or there might be a typing error in the list of protocols that you used for this function.");
		};
		for(auto attrIt=tempList.begin(); attrIt!=tempList.end(); attrIt++){
			attrIt->SetBioFlexBusId(bioFlexBusId);
		};
		attributeVec.insert(attributeVec.end(), tempList.begin(), tempList.end());
	}
	boost::shared_ptr<BfbClient> tempClient=boost::make_shared<BfbClient>(bioFlexBusId, sendMessageHandle, getIterationNumberHandle, attributeVec);
	return tempClient;
}

void CommunicationInterface::HandleExpiredResendTimer(const boost::system::error_code& error){
	if(!error){
		std::list< boost::shared_ptr<const ExtendedBfbMessage> > resendMessages;
		std::list< boost::shared_ptr<const ExtendedBfbMessage> > expiredMessages;
		boost::posix_time::time_duration deltaT;
		
		{// This bracket may seem unnecessary, but it makes shure that the lock that will be created in the very next line is released as soon as possible (when the corresponding '}'-bracket closes).
			boost::lock_guard<boost::mutex> lock(*UnansweredMessagesMutex);
			while(UnansweredMessages.size()>=1){
				auto currentTime=boost::posix_time::microsec_clock::local_time();
				deltaT= currentTime-UnansweredMessages.front()->TimeOfLastTransmission;
				if( deltaT.total_microseconds() > ( TimeToWaitForResponse.total_microseconds()-10 ) ){
					if(UnansweredMessages.front()->NumOfTransmissions<3){
						resendMessages.push_back(UnansweredMessages.front());
					}else{
						expiredMessages.push_back(UnansweredMessages.front());
					};
					UnansweredMessages.pop_front();
				}else{
					break;
				};
			};
		};
		for (auto it=resendMessages.begin(); it!=resendMessages.end(); it++){
			std::cout<<"Sending message again:"<<boost::posix_time::microsec_clock::local_time()<<std::endl;
			BfbFunctions::printMessage(*it);
			SendMessage(*it);
		};
		
		for (auto it=expiredMessages.begin(); it!=expiredMessages.end(); it++){
			(*it)->CallBackFunction(boost::shared_ptr<const BfbMessage>());
		};
		
		if(!UnansweredMessages.empty()){
			boost::posix_time::time_duration deltaT= (boost::posix_time::microsec_clock::local_time()-UnansweredMessages.front()->TimeOfLastTransmission);
			ResendTimer.expires_from_now( TimeToWaitForResponse-deltaT );
			ResendTimer.async_wait(boost::bind(&CommunicationInterface::HandleExpiredResendTimer, this, boost::asio::placeholders::error));
			IsResendTimerActive=true;
		}else{
			IsResendTimerActive=false;
		}
	}else{
		IsResendTimerActive=false;
	};
}

