#include <functional>
#include <stdlib.h>
#include <queue>

#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>
#include <boost/signals2.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/weak_ptr.hpp>

#include <BfbMessage.hpp>
#include "TcpServer.hpp"
#include "TcpConnection.hpp"


/****************************************************** TcpInterface method implementations *********************************************/
TcpServer::TcpServer(const unsigned short port):
		Work(*IoService),
		Acceptor(*IoService, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port)),
		TcpConnections(std::map<unsigned char, boost::shared_ptr<TcpConnection>>()){
	for(unsigned char i=192;i<224;i++){
		TcpConnections[i]=boost::shared_ptr<TcpConnection>();//Create a lot of Null pointers
	};
	StartAcceptConnections();
	std::function<void()> tempFunction=[this](){
		for(;;){
			try{
				IoService->run();
				break;
			}catch(const TcpConnectionUtilities::LostConnection& error){
				unsigned char temp=*boost::get_error_info<TcpConnectionUtilities::lostConnectionTcpId>(error);
				std::cout<<"The network connection to the client with the TCP-ID "<<std::dec<<int(temp)<<" ( "<<std::hex<<int(temp)<<" ) was closed."<<std::endl;
			}
		};
	};
	IoServiceThread=boost::make_shared<boost::thread>(tempFunction); 
}

TcpServer::~TcpServer(){
	IoService->stop();
	IoServiceThread->join();
};

void TcpServer::SetTcpConnectionBroadcastState(unsigned char tcpId, bool enableBroadcast){
	if(enableBroadcast){
		TcpConnectionBroadcastList.insert(tcpId);
	}else{
		TcpConnectionBroadcastList.erase(TcpConnectionBroadcastList.find(tcpId));
	};
}

bool TcpServer::GetTcpConnectionBroadcastState(unsigned char tcpId){
	return (TcpConnectionBroadcastList.count(tcpId)>0);
}


void TcpServer::BroadcastMessage(boost::shared_ptr< const BfbMessage > message){
	unsigned char destination=message->GetDestination();
	for(auto it=TcpConnectionBroadcastList.begin(); it!=TcpConnectionBroadcastList.end(); ){
		try{
			if(destination==*it){ // Broadcast the message only if the client is NOT the correct receiver. Since the message should be already sent to this client, this prevents double messages.
				TcpConnections[*it]->SendMessage(message);
			};
			it++;
		}catch(std::out_of_range& err){
			auto tempIt=it;
			it++;
			TcpConnectionBroadcastList.erase(tempIt);
		};
	};
};

void TcpServer::ForwardIncomingMessage(boost::shared_ptr< const BfbMessage > message){
	for(auto it=InputMessagesRouteList.begin(); it!=InputMessagesRouteList.end(); it++){
		(*it)(message);
	};
	if(TcpConnectionBroadcastList.size()>0){
		BroadcastMessage(message);
	};
}

void TcpServer::ForwardOutgoingMessage(boost::shared_ptr< const BfbMessage > message){
	for(auto it=OutputMessagesRouteList.begin(); it!=OutputMessagesRouteList.end(); it++){
		(*it)(message);
	};
	if(TcpConnectionBroadcastList.size()>0){
		BroadcastMessage(message);
	};
}

void TcpServer::RouteIncomingMessagesTo(boost::function<void (boost::shared_ptr<const BfbMessage>)> forwardFunction){
	//InputMessageSignal.connect(forwardFunction); // Connect the function to the notification signal.
	InputMessagesRouteList.push_back(forwardFunction);
	return;
};

void TcpServer::RouteOutgoingMessagesTo(boost::function<void (boost::shared_ptr<const BfbMessage>)> forwardFunction){
	//OutputMessageSignal.connect(forwardFunction); // Connect the function to the notification signal.
	OutputMessagesRouteList.push_back(forwardFunction); // Connect the function to the notification signal.
	return;
};

boost::function<void (boost::shared_ptr<const BfbMessage>)> TcpServer::GetSendMessageHandle(){
	return [&, this](boost::shared_ptr<const BfbMessage> outputMessage)->void{
		if(this){
			this->SendMessage(outputMessage);
		};
	};
};

void TcpServer::SendMessage(boost::shared_ptr<const BfbMessage> message){
	auto it=TcpConnections.find(message->GetDestination()); // Search for the TCP-client with the appropriate TCP-ID
	if(it!=TcpConnections.end() && it->second!=nullptr){ // If a client was found and it is not just a nullptr,...
		it->second->SendMessage(message); // forward the message to it's send method.
	};
	ForwardOutgoingMessage(message);
};

void TcpServer::StartAcceptConnections(){
		boost::shared_ptr<boost::asio::ip::tcp::socket> newSocket=boost::make_shared<boost::asio::ip::tcp::socket>(*IoService);
		Acceptor.async_accept( *newSocket,
			boost::bind(&TcpServer::HandleAcceptedConnection, 
				this, 
				newSocket,
				boost::asio::placeholders::error));
}

void TcpServer::NotifyOfNewConnection(std::function<void(unsigned char)> notificationFunction){
	NewConnectionNotificationFunctions.push_back(notificationFunction);
};

void TcpServer::HandleAcceptedConnection(boost::shared_ptr<boost::asio::ip::tcp::socket> newSocket,
	const boost::system::error_code& error){
	if (error){
		StartAcceptConnections();
		return;
	}
	std::map<unsigned char,boost::shared_ptr<TcpConnection>>::iterator it;
	for(it=TcpConnections.begin(); it!=TcpConnections.end(); ++it){ // Search for an unused TCP-ID. If one of the listed clients is inactive, it's TCP-ID will be reused.
		if(it->second==nullptr || !it->second->GetActivationState()){break;};
	}
	if(it!=TcpConnections.end()){ //If an unused TCP-ID was found
		std::cout<<"Established a new network connection. It will use the  TCP-ID "<<std::dec<< int(it->first)<<" ( "<<std::showbase<<std::hex<< int(it->first) <<" )."<<std::endl;
		std::function<void(boost::shared_ptr<const BfbMessage> message)> tempFunction= boost::bind(&TcpServer::ForwardIncomingMessage, this,_1);
		boost::shared_ptr<TcpConnection> newConnection=boost::make_shared<TcpConnection>(IoService, newSocket, it->first, tempFunction);
		TcpConnections.at(it->first)=newConnection;
		for(auto func=NewConnectionNotificationFunctions.begin(); func!=NewConnectionNotificationFunctions.end(); func++){
			(*func)(it->first);
		};
	}else{
		newSocket->close();
		std::cout<<"The server has too many open connection. No further connections can be established. Sorry!"<<std::endl;  
	};
	StartAcceptConnections();
}
