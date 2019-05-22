#ifndef TCPSERVER_HPP
#define TCPSERVER_HPP

// STL includes
#include <set>
#include <stdlib.h>

// Boost includes
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/signals2.hpp>
#include <boost/thread.hpp>
#include <boost/weak_ptr.hpp>

// Own header files
#include <BfbMessage.hpp>
//#include <TcpConnection.hpp>

class TcpConnection;

class TcpServer
{
	public:
		/** \brief The constructor method. 
		 * \param port The port it will listen on for connection attempts.
		 */
		TcpServer(unsigned short port);
		~TcpServer();
		
		/** \brief Use this method in order to send a message to the appropriate  TCP-client. */ 
		void SendMessage(boost::shared_ptr<const BfbMessage> Message); // non-blocking
		
		/** \brief When specified using this method, incoming messages (from the tcp-clients) will be routed to the passed function. The function must accept a shared_ptr to a BioFlexBus message. */
		void RouteIncomingMessagesTo(boost::function<void (boost::shared_ptr<const BfbMessage>)> forwardFunction);
		
		/** \brief When specified using this method, outgoing messages (to the tcp-clients) will be routed to the passed function. The function must accept a shared_ptr to a BioFlexBus message. */
		void RouteOutgoingMessagesTo(boost::function<void (boost::shared_ptr<const BfbMessage>)> forwardFunction);
		
		/** \brief This method returns a handle to the SendMessage-method of the instance. This makes it redundant to work with boost::bind in order to create a handle manually. */
		boost::function<void (boost::shared_ptr<const BfbMessage>)> GetSendMessageHandle();
		
		void SetTcpConnectionBroadcastState(unsigned char tcpId, bool enableBroadcast);
		bool GetTcpConnectionBroadcastState(unsigned char tcpId);
		
		void NotifyOfNewConnection(std::function<void(unsigned char)> notificationFunction);
	private:
		TcpServer(const TcpServer&) = delete;
		TcpServer & operator=(const TcpServer&) = delete;
		
		boost::shared_ptr<boost::asio::io_service> IoService=boost::make_shared<boost::asio::io_service>(); /*!< The service object managing all the asynchronous tasks. */
		boost::asio::io_service::work Work; /*!< The worker keeps the IoService object busy. Without it, the IOService sometimes runs out of work before the asynchronous receive operations are started and stops itself.*/
		boost::shared_ptr<boost::thread> IoServiceThread=boost::shared_ptr<boost::thread>(); /*!< Thread in which the IoService object runs. */
		boost::asio::ip::tcp::acceptor Acceptor; /*!< The Acceptor is responsible for the handling of connection attempts. */
		std::map<unsigned char, boost::shared_ptr<TcpConnection>> TcpConnections; /*!< This map holds pointers to the Connection instances of which each is managing one connection to a TCP client. The key is always the TCP ID (a number used for routing of messages) of the connection. */
		
		std::set<unsigned char> TcpConnectionBroadcastList;
		void BroadcastMessage(boost::shared_ptr<const BfbMessage> message);
		
		/** \brief Start to accept connection attempts from external programs via network.*/
		void StartAcceptConnections(); 
		
		/** \brief The method handles a successful connection establishment */
		void HandleAcceptedConnection(boost::shared_ptr<boost::asio::ip::tcp::socket> newSocket, const boost::system::error_code& error); 
		
		/** This signal is used to connect receivers (for example a printing function or another interface) to the tcp-clients. 
		 * Every time a tcp client receives a message, it will use this signal to inform all receivers.
		 * In order to add a receiver, the "RouteIncomingMessagesTo" function may be used.
		 */
		std::list<boost::function<void (boost::shared_ptr<const BfbMessage>)>> InputMessagesRouteList={};
		
		void ForwardIncomingMessage(boost::shared_ptr<const BfbMessage> message);
		
		/** This signal is used to connect receivers (for example a printing function or another interface) to the tcp-clients. 
		 * Every time a message is made ready for transmission, it will use this signal to inform all receivers.
		 * In order to add a receiver, the "RouteIncomingMessagesTo" function may be used.
		 */
		std::list<boost::function<void (boost::shared_ptr<const BfbMessage>)>> OutputMessagesRouteList={};
		void ForwardOutgoingMessage(boost::shared_ptr< const BfbMessage > message);
		
		std::list<std::function<void(unsigned char)>> NewConnectionNotificationFunctions={};
};
#endif 
