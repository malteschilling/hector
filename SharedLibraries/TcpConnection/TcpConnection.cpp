// STL includes
#include <functional>

// Boost includes
#include <boost/make_shared.hpp>
#include <boost/thread/mutex.hpp>

// Own header files
#include "TcpConnection.hpp"


/****************************************************** TcpConnection method implementations *********************************************/
TcpConnection::TcpConnection(boost::shared_ptr<boost::asio::io_service> ioService, 
							boost::shared_ptr<boost::asio::ip::tcp::socket> socket, 
							unsigned char TcpId, 
							std::function<void (boost::shared_ptr<const BfbMessage>)>& incomingMessageSignal):
			InputData(std::vector<unsigned char>(0)),
			OutputData(std::vector<unsigned char>(0)), 
			IoService(ioService),
			Socket(socket),
			TcpId(TcpId),
			MessagesToBeSent(std::queue<boost::shared_ptr<const BfbMessage>>()),
			IsSendPending(false),
			ConnectionMutex(boost::make_shared<boost::mutex>()),
			IncomingMessageFunctionCallback(incomingMessageSignal){
	boost::asio::ip::tcp::no_delay option(true);
	Socket->set_option(option);
	InputData.reserve(4096);
	OutputData.reserve(4096);
	boost::shared_ptr<boost::asio::io_service::work> Work;
	TryToReceiveMessages();
}


unsigned char TcpConnection::GetTcpId(){
	return TcpId;
}

bool TcpConnection::GetActivationState(){
	return IsActive;
}


void TcpConnection::TryToReceiveMessages(){
	InputData.resize(8);
	boost::asio::async_read(*Socket, boost::asio::buffer(InputData, 8),
		boost::bind(&TcpConnection::HandleReceivedShortPacketOrLongPacketHeader, this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
}

void TcpConnection::HandleReceivedShortPacketOrLongPacketHeader(const boost::system::error_code& error,
    size_t bytes_transferred){
	if (error){
		IsActive=false;
		throw TcpConnectionUtilities::LostConnection()<<TcpConnectionUtilities::lostConnectionTcpId(TcpId);
		return;
	}
	if(BfbFunctions::isValidShortPacket(InputData)){//Short packet
		boost::shared_ptr<BfbMessage> message(boost::make_shared<BfbMessage>(InputData));
		if(TcpId!=0){
			message->SetSource(TcpId);
		};
		IncomingMessageFunctionCallback(message);
		TryToReceiveMessages();
	}else if(BfbFunctions::isValidLongPacketHeader(InputData) || BfbFunctions::isValidUltraLongPacketHeader(InputData)){//Long packet
		unsigned long numOfMissingBytes=BfbFunctions::numOfMissingBytes(InputData);
		InputData.resize(numOfMissingBytes+bytes_transferred,0);
		boost::asio::async_read(*Socket, boost::asio::buffer(InputData.data()+bytes_transferred, numOfMissingBytes),
			boost::bind(&TcpConnection::HandleReceivedLongPacket, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
	}else{
		//resynchronize
	}
}

void TcpConnection::HandleReceivedLongPacket(const boost::system::error_code& error,
    size_t bytes_transferred){
	if (error){
		IsActive=false;
		throw TcpConnectionUtilities::LostConnection()<<TcpConnectionUtilities::lostConnectionTcpId(TcpId);
		return;
	}
	if( BfbFunctions::isValidLongPacket(InputData) || BfbFunctions::isValidUltraLongPacket(InputData)){//Long packet
		boost::shared_ptr<BfbMessage> message(boost::make_shared<BfbMessage>(InputData));
		if(TcpId!=0){
			message->SetSource(TcpId);
		};
		IncomingMessageFunctionCallback(message);
	}else{
		//resynchronize
	};
	TryToReceiveMessages();
}



void TcpConnection::SendMessage(boost::shared_ptr<const BfbMessage> message){
	boost::lock_guard<boost::mutex> lock(*ConnectionMutex); 
	MessagesToBeSent.push(message);
	if(!IsSendPending){
		SendNextMessage();
	};
};

void TcpConnection::SendNextMessage(){//Don't call this function without having locked the ConnectionMutex before. 
	boost::shared_ptr<BfbMessage> tempMessage(boost::make_shared<BfbMessage>(*(MessagesToBeSent.front())));
	MessagesToBeSent.pop();
	//if(TcpId!=2){
	//	tempMessage->SetDestination(2);
	//};
	OutputData=tempMessage->GetRawData();
	IsSendPending=true;
	size_t numOfBytesToTransfer=OutputData.size();
	if(numOfBytesToTransfer>pow(2,16)){
		SendPartialMessage(static_cast<boost::asio::error::basic_errors>(0), 0);
		return;
	};
	boost::asio::async_write(*Socket,
			boost::asio::buffer(OutputData.data(), numOfBytesToTransfer),
			boost::bind(&TcpConnection::HandleSentMessage, this,
				boost::asio::placeholders::error));
};

void TcpConnection::SendPartialMessage(const boost::system::error_code& error, long unsigned int bytes_transferred){
	if (error){
		IsActive=false;
		throw TcpConnectionUtilities::LostConnection()<<TcpConnectionUtilities::lostConnectionTcpId(TcpId);
		return;
	}
	size_t numOfBytesToTransfer=OutputData.size()-bytes_transferred;
	if(numOfBytesToTransfer>pow(2,14)){
		numOfBytesToTransfer=pow(2,14);
		boost::asio::async_write(*Socket,
			boost::asio::buffer(OutputData.data()+bytes_transferred, numOfBytesToTransfer),
			boost::bind(&TcpConnection::SendPartialMessage, this,
				boost::asio::placeholders::error, bytes_transferred+numOfBytesToTransfer));  
	}else{
		boost::asio::async_write(*Socket,
			boost::asio::buffer(OutputData.data()+bytes_transferred, numOfBytesToTransfer),
			boost::bind(&TcpConnection::HandleSentMessage, this,
				boost::asio::placeholders::error));  
	}
}



void TcpConnection::HandleSentMessage(const boost::system::error_code& error){
	if (error){
		IsActive=false;
		throw TcpConnectionUtilities::LostConnection()<<TcpConnectionUtilities::lostConnectionTcpId(TcpId);
		return;
	}
	boost::lock_guard<boost::mutex> lock(*ConnectionMutex); 
	if(MessagesToBeSent.empty()){
		IsSendPending=false;
	}else{
		SendNextMessage();
	};
}