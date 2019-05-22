#ifndef THREADSAFEQUEUE_HPP
#define THREADSAFEQUEUE_HPP

// STL includes
#include <mutex>
#include <queue>

// Boost includes
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/function.hpp>
#include <boost/thread/condition_variable.hpp>

template <typename Type>
class ThreadSafeQueue
{
	private:
		/** \brief Queue in which the elements are buffered. */
		std::queue<Type> Queue;
		
		/** \brief Mutex that is used to prevent multiple simultaneous access to the queue. */
		boost::mutex AccessMutex;
		
		/** \brief If another thread tries to retrieve an element from the queue (blocking) when the queue is empty, it will be notified as soon as the element is  appended using this variable. */
		boost::condition_variable NewEntry;
		
	public:		
		ThreadSafeQueue(const ThreadSafeQueue&) = delete;
		ThreadSafeQueue & operator=(const ThreadSafeQueue&) = delete;
		
		ThreadSafeQueue():
			Queue(),
			AccessMutex(),
			NewEntry(){
		};
		
		/** \brief This method inserts a new element into the queue. */
		void Push(Type newEntry){
			std::lock_guard<boost::mutex> lock(AccessMutex); 
			Queue.push(newEntry);
			NewEntry.notify_one();
			return;
		};
		
		/** \brief The method returns a handle to the push method of an instance of ThreadSafeQueue. */
		boost::function<void (Type)> GetPushHandle(){
			return [&, this](Type newEntry)->void{
				if(this){
					this->Push(newEntry);
				};
			};
		};
		
		/** \brief This method tests whether the queue is empty. */
		bool IsEmpty(){
			return Queue.empty();
		}
		
		/** \brief This method tests whether the queue is not empty. */
		bool IsNotEmpty(){
			return !( Queue.empty() );
		}

		/** \brief This method tries to retrieve an element from the queue (blocking) until the specified system time is reached.
		 * In case the queue is empty when the method is called, the method will block for the specified period. If no entry was added within this period, the function will throw a range_error exception.
		 * To make sure that the method will not throw, test beforehand whether the queue contains elements using the IsEmpty method.
		 */
		Type PopUntil(boost::system_time absTime){
			boost::mutex::scoped_lock lock(AccessMutex); 
			if(IsEmpty()){
				if(!NewEntry.timed_wait(lock, absTime, boost::bind(&ThreadSafeQueue::IsNotEmpty, this))){
					throw std::range_error("Trying to retrieve an element from an empty queue.");
				}
			};
			Type tempElement=Queue.front();
			Queue.pop();
			return tempElement;
		};
		
		Type PopWithin(boost::posix_time::time_duration timeDuration){
			return PopUntil(boost::get_system_time()+timeDuration);
		};
		/** \brief This method tries to retrieve an element from the queue. The method does not block.
		 */
		Type Pop(){
			return PopUntil(boost::get_system_time());
		};
};
#endif