#ifndef SERIALINTERFACE_HPP
#define SERIALINTERFACE_HPP

// STL includes
#include <stdlib.h>
#include <list>
#include <map>
#include <queue>
#include <vector>

// Own header files
#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/signals2.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/weak_ptr.hpp>

// Own header files
#include <BfbMessage.hpp>
//#include "TcpInterface.hpp"

//Forward declarations (The "real" declaration is in the 'CommunicationInterface.cpp' file.)
class SerialConnection;
//class SerialInterfaceKey;



class SerialInterface
{
	public:
		/** \brief The constructor takes no arguments. It searches for all connected bus masters and creates connections to them. There's currently no way to let it search for a specific one or to exclude one from the search. */
		SerialInterface();
		~SerialInterface();
		
		/** \brief Use this method in order to send a message to one of the connected serial clients. The destination ID of the client must be specified in the message. */
		void SendMessage(boost::shared_ptr<const BfbMessage> message);
		
		/** \brief This method can be used to route incoming messages to the passed function. The function must accept a shared_ptr to a BioFlexBus message. It is possible to call this method multiple times passing different functions. In this case, the incoming message will be distributed to all passed functions. */
		void RouteIncomingMessagesTo(boost::function<void (boost::shared_ptr<const BfbMessage>)> forwardFunction);
		
		/** \brief The method returns a sorted list of clients connected via the serial interface. 
		 * \return A sorted list of clients that can be reached via the serial interface. 
		 */
		std::list<unsigned char> GetConnectedClients(); 
		std::list<std::string> GetSerialPortNames();
		
		/** \brief This method returns a handle to the SendMessage-method of the instance. This makes it redundant to work with boost::bind in order to create a handle manually. */
		boost::function<void (boost::shared_ptr<const BfbMessage>)> GetSendMessageHandle();
		
		void SetNumOfTransmissionAttempts(unsigned int numOfTransmissionAttempts);
	private:
		SerialInterface(const SerialInterface&) = delete;
		SerialInterface & operator=(const SerialInterface&) = delete;
		
		boost::shared_ptr<boost::asio::io_service> IoService=boost::make_shared<boost::asio::io_service>(); /*!< The service object managing all the asynchronous tasks */
		boost::asio::io_service::work Work; /*!< The worker keeps the IoService object busy. Without it, the IOService is sometimes runs out of work before the asynchronous receive operations are started and stops itself.*/
		boost::thread IoServiceThread; /*!< Thread in which the IoService object runs it's run method. */
		/*!< Signal used to distribute an incoming message. Multiple recipients may be connected to this signal using the "RouteIncomingMessagesTo" method. */
		void ForwardIncomingMessage(boost::shared_ptr<const BfbMessage> message);
		std::list<boost::function<void (boost::shared_ptr<const BfbMessage>)>> InputMessagesRouteList={};

		std::vector<boost::shared_ptr<SerialConnection>> SerialConnections; /*!< This vector is used to store the serial connections to all available serial ports.*/
		std::map<unsigned char, boost::shared_ptr<SerialConnection>> Clients; /*!< Map that stores all connected client IDs and the corresponding serial connection instances. A message that is addressed to a certain client may be routed to the serial connection registered for this client ID. */
		
		unsigned int NumOfTransmissionAttempts=3;
};

#endif /* COMMUNICATION_INTERFACE_HPP_INCLUDED */
