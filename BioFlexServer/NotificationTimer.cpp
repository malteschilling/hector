// STL includes
#include <algorithm>
#include <iterator>
#include <list>

// Boost includes
#include <boost/asio.hpp>
#include <boost/date_time.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

// Own header files
#include "NotificationTimer.hpp"
#include <BfbMessage.hpp>
#include "BfbProtocolIds.hpp"


/** \brief The class represents one programm that is connected via TCP with the server. It is responsible for storing the time at which the process wants to be  woken up.
 * 	In order to have the same interface between the server and the simulation, a control program is supposed to send a message that tells the server/simulation that 
 * 	the program wants to be informed when a certain time has passed. On the server side this is needless since all involved programs have to operate in real time. 
 * 	Using the simulation, however, it is not necessary to operate with real time, but with simulation time instead. This might be the case if the simulation time runs 
 * 	slower/faster than the real time. In this case, the programs must be informed when the specified time has passed. Therefore, each program is represented by an 
 * 	instance of this class. It keeps track of the timing commands sent from the programm. 
 */
class Process
{
	public:
		Process(unsigned int tcpId);
		void ResetTimer();
		void AddTime(double seconds);
		void AddTime(boost::posix_time::time_duration deltaT);
		boost::posix_time::ptime GetNextUpdateTime();
		unsigned int GetTcpId();
	private:
		boost::posix_time::ptime NextUpdateTime;
		unsigned int TcpId;
};


Process::Process(unsigned int tcpId):
		NextUpdateTime(boost::posix_time::microsec_clock::local_time()),
		TcpId(tcpId){
}

void Process::ResetTimer(){
	NextUpdateTime=boost::posix_time::microsec_clock::local_time();
};

void Process::AddTime(double seconds){
	if(seconds<0){
		seconds=0;
	};
	NextUpdateTime+=boost::posix_time::milliseconds(static_cast<unsigned int>(seconds*1000));
}
void Process::AddTime(boost::posix_time::time_duration deltaT){
	if(deltaT.is_negative()){
		return;
	};
	NextUpdateTime+=deltaT;
}

boost::posix_time::ptime Process::GetNextUpdateTime(){
	return NextUpdateTime;
};

unsigned int Process::GetTcpId(){return TcpId;};


NotificationTimer::NotificationTimer(boost::function<void (boost::shared_ptr<const BfbMessage>)> sendMessageHandle):
	SendMessageHandle(sendMessageHandle),
	IoServiceThread(boost::bind(&boost::asio::io_service::run,&IoService)), // Start the IO-handler needed for communication.
	Work(IoService),
	AsyncTimer(IoService),
	Processes(std::map<unsigned int, boost::shared_ptr<Process>>()),
	OutstandingNotifications(std::list<boost::shared_ptr<Process>>()),
	SingleAccessMutex(new boost::mutex){
}


NotificationTimer::~NotificationTimer(){
	IoService.stop();
	IoServiceThread.join();
}


void NotificationTimer::ResetTimer(unsigned char TcpId){
	try{
		auto tempProcess=Processes.at(TcpId);
		tempProcess->ResetTimer();
	}catch(const std::out_of_range& oor){
	
		
	};
}


void NotificationTimer::ProcessMessage(boost::shared_ptr<const BfbMessage > message){
	bool resetTimer=false;
	if(message->GetDestination()==TimerId && message->GetProtocol()==BfbProtocolIds::SIMSERV_1_PROT){
		boost::lock_guard<boost::mutex> lock(*SingleAccessMutex); 
		boost::shared_ptr<Process> tempProcess;
		try{
			tempProcess=Processes.at(message->GetSource());
		}catch(const std::out_of_range& oor){
			Processes.insert(std::pair<unsigned int, boost::shared_ptr<Process>>(message->GetSource(), boost::make_shared<Process>(Process(message->GetSource()))));
			tempProcess=Processes.at(message->GetSource());
		};
		switch(message->GetCommand()){
			case 0:{
				tempProcess->ResetTimer();
			};
			case 12:{
				// Set Timer
				auto payload=message->GetPayload();
				double seconds=0;
				for(unsigned int i=0;i<payload.size();i++){
					seconds+=double(payload[i])/1000*pow(256,i);
				};
				boost::posix_time::time_duration deltaT=boost::posix_time::millisec((seconds*1000));
				tempProcess->AddTime(deltaT);
				
				// If there's currently an outstanding notification in the list for this process, it will be deleted in favor of the new notification request.
				OutstandingNotifications.remove(tempProcess);
				
				/*!< Next, the notification must be inserted into the list at the appropriate position. If the notification time of this process
				 * is later than the last element, it should be 'push_back'ed. If the notification time is less than the first element, it should be
				 * 'push_front'ed. In order to reduce the number of conditional clauses, a reverse iterator to the last element is created.
				 * This iterator is then incremented (since it's a reverse iterator, every increment moves it further to the front) until either the 
				 * first element has been exceeded or the next update time of an element is bigger than
				 * that of the current process.  
				 * This whole process could have been easier if a normal iterator (front->back) was used. But the chances are high that the new notification
				 * should be executed at last (later than the already planned notifications).
				 */
				std::list<boost::shared_ptr<Process>>::reverse_iterator iter;
				for(iter=OutstandingNotifications.rbegin(); iter!=OutstandingNotifications.rend() && (*(iter))->GetNextUpdateTime()<tempProcess->GetNextUpdateTime();iter++){};
				/*!< The iterator points to an element that is one position further to the front than the position the new notification should be placed.
				 * Therefore, the iterator must be decremented before use. BUT since the iterator is of the reverse type, it can not be used directly in the insert-method
				 * of the list. This method accepts only normal iterators. Therefore, it must be converted using its base-method first. This, however, changes the position
				 * the now normal iterator points to (see http://www.drdobbs.com/cpp/three-guidelines-for-effective-iterator/184401406?pgno=3 ).
				 */
				if(iter==OutstandingNotifications.rend()){
					/*!< If the iterator points now to the very first element, obviously, the newly inserted notification is the one that should be processed earliest.
					* Therefore, the asynchronous timer should be cancelled and newly set.
					*/
					OutstandingNotifications.push_front(tempProcess);
					resetTimer=true;
				}else{
					OutstandingNotifications.insert(iter.base(),tempProcess);
				};
			};
		}
	}
	if(resetTimer){
		//AsyncTimer.expires_at(OutstandingNotifications.front()->GetNextUpdateTime());
		boost::posix_time::time_duration deltaT= (OutstandingNotifications.front()->GetNextUpdateTime()-boost::posix_time::microsec_clock::local_time());
		AsyncTimer.expires_from_now( deltaT );
		AsyncTimer.async_wait(boost::bind(&NotificationTimer::HandleExpiredTimer, this, boost::asio::placeholders::error));
	};
	return;
}

boost::function<void (boost::shared_ptr<const BfbMessage>)>  NotificationTimer::GetProcessMessageHandle()
{
	return [&, this](boost::shared_ptr<const BfbMessage> message)->void{
		if(this){
			this->ProcessMessage(message);
		};
	};
}

void NotificationTimer::HandleExpiredTimer(const boost::system::error_code& error){
	if( error != boost::asio::error::operation_aborted){
		boost::lock_guard<boost::mutex> lock(*SingleAccessMutex); 
		while(!(OutstandingNotifications.empty()) && (OutstandingNotifications.front()->GetNextUpdateTime())<=boost::posix_time::microsec_clock::local_time()){
			// Create a reply message
			boost::shared_ptr<BfbMessage> reply=boost::make_shared<BfbMessage>();
			reply->SetDestination(OutstandingNotifications.front()->GetTcpId() );
			reply->SetSource(TimerId);
			reply->SetBusAllocationFlag(false);
			reply->SetProtocol(BfbProtocolIds::SIMSERV_1_PROT);
			reply->SetCommand(13);
			SendMessageHandle(reply);
			OutstandingNotifications.pop_front();
		};
		if(!(OutstandingNotifications.empty())){
			boost::posix_time::time_duration deltaT= (OutstandingNotifications.front()->GetNextUpdateTime()-boost::posix_time::microsec_clock::local_time());
			AsyncTimer.expires_from_now( deltaT );
			//AsyncTimer.expires_at(OutstandingNotifications.front()->GetNextUpdateTime());
			AsyncTimer.async_wait(boost::bind(&NotificationTimer::HandleExpiredTimer, this, boost::asio::placeholders::error));
		}
	};
};
