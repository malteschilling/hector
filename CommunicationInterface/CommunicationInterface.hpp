#ifndef COMMUNICATIONINTERFACE_HPP
#define COMMUNICATIONINTERFACE_HPP

// STL includes
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <queue>
#include <stdlib.h>
#include <string.h>
#include <typeinfo>

// Boost includes
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/assign/ptr_list_of.hpp>
#include <boost/assign.hpp>
#include <boost/bind.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/date_time.hpp>
#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/program_options.hpp>
#include <boost/weak_ptr.hpp>

// Own header files
#include "BfbClient.hpp"
#include <BfbMessage.hpp>
#include "CommunicationXmlParser.hpp"
#include <TcpConnection.hpp>


#ifndef SWIG
class ExtendedBfbMessage: public BfbMessage{
	public:
		ExtendedBfbMessage(const BfbMessage message):
			BfbMessage(message){
		};
		mutable unsigned char 		NumOfTransmissions=0;
		mutable boost::system_time 	TimeOfLastTransmission=boost::get_system_time();
		mutable std::function<void (boost::shared_ptr<const BfbMessage>)> CallBackFunction=nullptr;
};
#endif

class CommunicationInterface{
	private:
	#ifndef SWIG
		boost::shared_ptr<boost::asio::io_service> IoService; /*!< The service object managing all the asynchronous tasks. */
		boost::shared_ptr<boost::asio::io_service::work> Work; /*!< The worker keeps the IoService object busy. Without it, the IOService sometimes runs out of work before the asynchronous receive operations are started and stops itself.*/
		boost::thread IoServiceThread; /*!< Thread in which the IoService object runs. */
		const boost::shared_ptr<boost::mutex>	UnansweredMessagesMutex;
		
		void HandleIncomingMessage(boost::shared_ptr<const BfbMessage> message);
		boost::shared_ptr<TcpConnection> TcpConn=boost::shared_ptr<TcpConnection>();
	
		CommunicationXmlParser::AttributeMap DefaultAttributes=CommunicationXmlParser::AttributeMap();
		boost::circular_buffer<boost::shared_ptr<const BfbMessage>> UnassignableMessages=boost::circular_buffer<boost::shared_ptr<const BfbMessage>>(100);
	
		std::list< boost::shared_ptr<const ExtendedBfbMessage> > UnansweredMessages=std::list< boost::shared_ptr<const ExtendedBfbMessage> >();
		unsigned long int IterationNumber=1;
		
		/** \brief The method handles the expiration of the resend timer. This typically means that a reply was not received although one was expected. */
		void HandleExpiredResendTimer(const boost::system::error_code& error);
		boost::asio::deadline_timer ResendTimer; /*!< Timer instance is used to make sure that an answer was received within a certain time. If this time is exceeded, the request will be resent.*/
		bool IsResendTimerActive=false; /*!< Flag that is used to tell whether an asynchronous wait has been configured for the ResendTimer.*/
		boost::posix_time::time_duration TimeToWaitForResponse=boost::posix_time::seconds(1); /*!> The maximum time between a request and the corresponding reply before the request is resent.*/
		
		CommunicationInterface(const CommunicationInterface&) = delete;
		CommunicationInterface & operator=(const CommunicationInterface&) = delete;
		#else
		CommunicationInterface(const CommunicationInterface&);
		CommunicationInterface & operator=(const CommunicationInterface&);
		#endif
	public: 
		CommunicationInterface(std::string tcpIp="localhost", std::string portNum="50002");
		~CommunicationInterface();
		boost::shared_ptr<BfbClient > CreateBfbClient(short unsigned int bioFlexBusId, std::vector< std::string > protocols);
		/** \brief Use this method in order to send a message to the TCP-connection. */ 
		void SendMessage(boost::shared_ptr< const BfbMessage > message, std::function<void (boost::shared_ptr<const BfbMessage>)> replyHandler=nullptr);
		unsigned long int GetIterationNumber();
		void NotifyOfNextIteration();
		void ParseProtocolXmls(std::vector<std::string> xmls);
		void ParseProtocolXml(std::string xml);
	#ifndef SWIG
		void ParseProtocolXml(std::istream stream);
	#endif
};

#endif