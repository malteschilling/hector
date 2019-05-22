#ifndef NOTIFICATIONTIMER_HPP
#define NOTIFICATIONTIMER_HPP

// STL includes
#include <stdlib.h>
#include <map>

// Boost includes
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

// Own header files
#include <BfbMessage.hpp>

// forward declaration
class Process;

const unsigned int TimerProtocolId=12;

class NotificationTimer
{
	public:
		NotificationTimer(boost::function<void (boost::shared_ptr<const BfbMessage>)> sendMessageHandle);
		~NotificationTimer();
		void ProcessMessage(boost::shared_ptr<const BfbMessage> message);
		/** \brief This method returns a handle to the SendMessage-method of the instance. This makes it redundant to work with boost::bind in order to create a handle manually. */
		boost::function<void (boost::shared_ptr<const BfbMessage>)> GetProcessMessageHandle();
		void ResetTimer(unsigned char TcpId);
	private:
		NotificationTimer(const NotificationTimer&) = delete;
		NotificationTimer & operator=(const NotificationTimer&) = delete;
		
		const unsigned int TimerId=14;
		boost::function<void (boost::shared_ptr<BfbMessage>)> SendMessageHandle;

		boost::asio::io_service IoService; /*!< Asynchronous communication handler used by the instance to connect to the socket and to call the handler methods. */

		boost::thread IoServiceThread; /*!< Thread in which the IoService object runs it's run method. */
		boost::asio::io_service::work Work;
		boost::asio::deadline_timer AsyncTimer; /*!< Timer that is used for client timing purposes. If a client must run with a certain frequency, a defined message can be used to configure the timer such that it will expire after the desired time interval. Then, the "HandleTimer" method will be called. This is helpful if a client should run both with a non-realtime simulation and with this interface that is connected to a real (and therefore realtime) system. */

		std::map<unsigned int, boost::shared_ptr<Process>> Processes;
		std::list<boost::shared_ptr<Process>> OutstandingNotifications;
		/** \brief This method will be called if the timer expires. It will then send a message to notify the connected client. */  
		void HandleExpiredTimer(const boost::system::error_code& error);
		
		/** \brief This method should be called if a message was received that contains timer-specific commands.*/
		//void HandleTimingRequest(boost::shared_ptr<BfbMessage> message);
		//boost::shared_ptr<boost::asio::io_service::work> Work; /*!< The worker keeps the IoService object busy. Without it, the IOService is sometimes runs out of work before the asynchronous receive operations are started and stops itself.*/
		/** \brief Mutex that is used to prevent multiple simultaneous access to the queue. */
		boost::shared_ptr<boost::mutex> SingleAccessMutex;
};
#endif