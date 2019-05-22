#ifndef TCPCONNECTION_HPP
#define TCPCONNECTION_HPP

// STL includes
#include <functional>
#include <stdlib.h>
#include <string>
#include <queue>

// Boost includes
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/exception/all.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/signals2.hpp>
#include <boost/thread.hpp>
#include <boost/weak_ptr.hpp>

// Own header files
#include <BfbMessage.hpp>

namespace TcpConnectionUtilities{
	typedef boost::error_info<struct blah, unsigned char> lostConnectionTcpId; 
	struct LostConnection: virtual std::exception, virtual boost::exception { };
};

class TcpConnection
{
	public:
		/** \brief The constructor 
		 * \param ioService The connection handler for asynchronous communication
		 * \param socket The socket this TCP-client is supposed to use for communication
		 * \param tcpId The internally used ID that is used route messages to the serial clients and back towards the corresponding TCP-client. Every TCP-client gets an unique ID.
		 * \param incomingMessageSignal This is used for the signaling of received messages. Modules that should be informed about a received message must be connected to this signal.
		 */ 
		TcpConnection(boost::shared_ptr<boost::asio::io_service> ioService, boost::shared_ptr<boost::asio::ip::tcp::socket> socket, unsigned char tcpId, std::function<void (boost::shared_ptr<const BfbMessage>)>& incomingMessageFunction);
		
		/** \brief Method responsible for sending messages via the TCP-socket that was assigned to an instance of this class.
		 * \param message The message that should be send.
		 */
		void SendMessage(boost::shared_ptr<const BfbMessage> message);
		
		/** \brief Get the TCP-ID of the instance
		 * \return The TCP-ID is an unique number used for routing of messages on the serial side of the communication
		 */
		unsigned char GetTcpId();
		
		/** \brief Test whether the connection is still active/the socket was closed.*/
		bool GetActivationState();

	private:
		TcpConnection(const TcpConnection&) = delete;
		TcpConnection & operator=(const TcpConnection&) = delete;
		
		/** \brief Method that must be called in order to start the receiving automatism.
		 * This is done in the constructor once. It is then called whenever one message has been received completely and a new message should be received.
		 */ 
		void TryToReceiveMessages();
		
		/**  \brief This method should be called by the asynchronous IO-Handler whenever the first eight bytes of a message have been received. Dependend on whether the received bytes represent a complete short message or only the first part of a long message, the message-receipt will be signaled and the message receival automatism will be restarted or - for the latter case - the asynchronous communication framework will be configured in order to receive the number of bytes that is missing to construct a long message. */
		void HandleReceivedShortPacketOrLongPacketHeader(const boost::system::error_code& error,
			size_t bytes_transferred);

		/** \brief The "HandleReceivedShortMessageOrLongHeader" configures the asynchronous interface such that the missing number of bytes will be received in case only the header of a long package was received. After they were received, this function must be called. It signals the receipt of a message to the connected receivers and restarts the receival automatism. */
		void HandleReceivedLongPacket(const boost::system::error_code& error, size_t bytes_transferred);
		
		/** \brief The "SendMessage" method does not send the messages directly. It merely pushes them into a queue. Afterwards, it will call this method that is responsible for configuring the asynchronous interface such that after the topmost message has been sent, the corresponding handler ("HandleSentMessage") will be called that calls this function again to prepare the next message for sending. In this way, all messages that are in the queue will be sent one after another.*/
		void SendNextMessage();
		
		/** \brief This method will be called whenever a message has been sent. It will then call the "SendNextMessage" method in order to prepare the next message for sending.*/
		void HandleSentMessage(const boost::system::error_code& error);

		/** \brief This method will be called whenever a message has been sent. It will then call the "SendNextMessage" method in order to prepare the next message for sending.*/
		void SendPartialMessage(const boost::system::error_code& error, unsigned long int bytes_transferred);
		
		bool IsActive=true; /*!< This variable represents the status of the TCP connection. If it is true, messages can be send and received. If it is false, the connection has been closed by the receiver and therefore no communication is possible. */ 
		
		std::vector<unsigned char> InputData; /*!< This variable holds the received bytes until a complete message has been received and it can be converted into an appropriate object. It should only be used in the handler methods! If the bytes are modified inbetween, the message will be undecipherable! */
		std::vector<unsigned char> OutputData; /*!< This variable holds the to-be-send bytes. Again: Do not modify the contents except for within the corresponding handler methods! */
		
		boost::shared_ptr<boost::asio::io_service> IoService; /*!< Asynchronous communication handler used by the instance to connect to the socket and to call the handler methods. */

		boost::shared_ptr<boost::asio::ip::tcp::socket> Socket; /*!< The assigned TCP-socket used for communication. */
		unsigned char TcpId; /*!< The assigned ID of a TCP-client. It is used for message routing. */
		
		std::queue<boost::shared_ptr<const BfbMessage>> MessagesToBeSent; /*!< Queue in which all messages are teporarily saved before they are sent.*/
		bool IsSendPending; /*!< Status variable signaling whether a message is waiting to be sent completely.*/
		boost::shared_ptr<boost::mutex> ConnectionMutex; /*!< This mutex is used to make sure only one thread accesses the send methods at the same time. */
		std::function<void (boost::shared_ptr<const BfbMessage>)> IncomingMessageFunctionCallback; /*!> In this variable, the reference to the signaling function is saved. The corresponding signla will be called every time a message was received. */
};
#endif