#ifndef ATTRIBUTE_HPP
#define ATTRIBUTE_HPP

// STL includes
#include <functional>
#include <limits>

// Boost includes
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/composite_key.hpp>
#include <boost/multi_index/hashed_index.hpp> 
#include <boost/multi_index/member.hpp> 
#include <boost/multi_index/mem_fun.hpp>
#include <boost/thread.hpp>
#include <boost/variant.hpp>

// Own header files
#include <BfbMessage.hpp>


namespace AttributeUtilities{
typedef std::vector<boost::variant<double, std::string> > variableTypeList;
typedef std::function<void(boost::shared_ptr< const BfbMessage > message, std::function<void (boost::shared_ptr<const BfbMessage>)> callBackFunction)> sendMessageHandle_t;
enum DataType_t{
	STRING,
	BOOL,
	UINT8,
	UINT16,
	UINT32,
	UINT64,
	INT8,
	INT16,
	INT32,
	INT64,
	FLOAT
};
static const std::map<std::string, DataType_t> StringDataTypeMap={	
							{"string", STRING}, 
							{ "bool",  BOOL},
							{ "uint8",  UINT8},
							{"uint16", UINT16},
							{"uint32", UINT32},
							{"uint64", UINT64},
							{  "int8",   INT8},
							{ "int16",  INT16},
							{ "int32",  INT32},
							{ "int64",  INT64},
							{ "float", FLOAT}};

class Entry{
	public:
		Entry(	DataType_t clientType,
			double clientLower,
			double clientUpper,
			const DataType_t hostType,
			const double hostLower,
			double hostUpper,
			bool limit=false):
			ClientType(clientType),
			ClientLower(clientLower),
			ClientUpper(clientUpper),
			HostType(hostType),
			HostLower(hostLower),
			HostUpper(hostUpper),
			Limit(limit){
		};
		DataType_t ClientType;
		double ClientLower;
		double ClientUpper;
		
		DataType_t HostType;
		double HostLower;
		double HostUpper;
		
		bool Limit=false;
		
		Entry(const Entry &entry):
			ClientType(entry.ClientType),
			ClientLower(entry.ClientLower),
			ClientUpper(entry.ClientUpper),
			HostType(entry.HostType),
			HostLower(entry.HostLower),
			HostUpper(entry.HostUpper),
			Limit(entry.Limit){
		};
		#ifndef SWIG 
		Entry& operator= (const Entry& entry){
			if (this != &entry){
				ClientType=entry.ClientType;
				ClientLower=entry.ClientLower;
				ClientUpper=entry.ClientUpper;
				HostType=entry.HostType;
				HostLower=entry.HostLower;
				HostUpper=entry.HostUpper;
				Limit=entry.Limit;
			};
			return *this;
		};
		#endif
};
#ifndef SWIG
enum AttributeState{
	OutOfDate=0,
	WaitingForRequestReply,
	WaitingForTransmitReply,
	UpToDate
};
#endif
};

class Attribute{
	private:
		#ifndef SWIG
		std::string 		Name;
		AttributeUtilities::AttributeState	State;
		unsigned char		BioFlexBusId;
		unsigned char 		ProtocolId;
		unsigned char 		RequestId;
		std::vector<AttributeUtilities::Entry> Entries;
		bool 			AutoConfirmTransmit=false;
		bool			IsTransmittable;
		AttributeUtilities::variableTypeList 	Value;
		unsigned long int 	LastUpdateIteration=0;
		long int 		MaxAge;
		double			TimeToWaitForAnswer;
		boost::shared_ptr<boost::mutex>		SingleAccessMutex;
		boost::shared_ptr<boost::condition_variable_any> 	StateChangeNotification;
		AttributeUtilities::sendMessageHandle_t SendMessage=[](boost::shared_ptr< const BfbMessage > message, std::function<void (boost::shared_ptr<const BfbMessage>)> callBackFunction)->void{};
		std::function<unsigned long int()> GetIterationNumber=[]()->unsigned long int{return 0;};
		std::function<void()> NotifyOfError=[](){};
		boost::shared_ptr<const BfbMessage> Request;
		#endif
	public:
		Attribute(	unsigned char bioFlexBusId,
				std::string name, 
				unsigned char protocolId, 
				unsigned char commandId, 
				const std::vector<AttributeUtilities::Entry> entries, 
				AttributeUtilities::sendMessageHandle_t sendMessage, 
				std::function<unsigned long int()> getIterationNumber,//=[](){return 0;}, 
				const bool isTransmittable=false, 
				const bool confirmTransmit=false,
				unsigned long int maxAge=0,
				double timeToWaitForAnswer=std::numeric_limits< double >::quiet_NaN());
		Attribute(const Attribute &attribute);
		#ifndef SWIG 
		Attribute & operator=(const Attribute& other);
		#endif
		//virtual 
		AttributeUtilities::variableTypeList GetValue(bool getNewValue=false);
		//virtual 
		void UpdateValue();
		void UpdateValueIfTooOld();
		void UpdateValueIfTooOldAndWait();
		//virtual 
		void SetValue(AttributeUtilities::variableTypeList value);
		void SetValue(AttributeUtilities::variableTypeList value, bool blocking);

		std::string GetName() const;
		void SetName(std::string name);
		unsigned char GetProtocolId() const;
		void SetProtocolId(unsigned char protocolId);
		unsigned char GetCommandId() const;
		void SetCommandId(unsigned char commandId);
		unsigned char GetBioFlexBusId();
		void SetBioFlexBusId(unsigned char bioFlexBusId);
		
		void SetIterationNumberHandle(std::function<unsigned long int()> getIterationNumber);
		void SetSendMessageHandle(AttributeUtilities::sendMessageHandle_t sendMessage);
		void SetNotifyOfErrorHandle(std::function<void()> notifyOfError);
		
		const std::vector<AttributeUtilities::Entry> GetEntries() const;
		
		//virtual 
		boost::shared_ptr<BfbMessage> CreateRequest();
		//virtual 
		void HandleRequestReply(boost::shared_ptr<const BfbMessage > message);

		//virtual 
		boost::shared_ptr<BfbMessage> CreateTransmit(AttributeUtilities::variableTypeList value);
		//virtual 
		void HandleTransmitReply(boost::shared_ptr<const BfbMessage > message);
};

#endif