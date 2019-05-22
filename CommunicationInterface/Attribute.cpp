// STL includes
#include <algorithm>

// Boost includes
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/tuple/tuple.hpp>

// Own header files
#include "Attribute.hpp"

using namespace AttributeUtilities;

Attribute::Attribute(	unsigned char bioFlexBusId,
			std::string name, 
			unsigned char protocolId, 
			unsigned char commandId, 
			const std::vector<AttributeUtilities::Entry> entries, 
			AttributeUtilities::sendMessageHandle_t sendMessage, 
			std::function<unsigned long int()> getIterationNumber, 
			const bool isTransmittable, 
			const bool confirmTransmit,
			unsigned long int maxAge,
			double timeToWaitForAnswer):
		Name(name),
		State(OutOfDate),
		BioFlexBusId(bioFlexBusId),
		ProtocolId(protocolId),
		RequestId(commandId),
		Entries(entries),
		AutoConfirmTransmit(confirmTransmit),
		IsTransmittable(isTransmittable),
		Value(AttributeUtilities::variableTypeList()),
		MaxAge(maxAge),
		TimeToWaitForAnswer(timeToWaitForAnswer),
		SingleAccessMutex(new boost::mutex),
		StateChangeNotification(new boost::condition_variable_any),
		SendMessage(sendMessage),
		GetIterationNumber(getIterationNumber),
		Request(CreateRequest())
		{
			/*
			if(Name=="setTimer"){
				std::cout<<"Created new 'setTimer:"<<std::endl;
				std::cout<<"IsTransmittable is set to: "<<int(IsTransmittable)<<std::endl;
				std::cout<<"AutoConfirmTransmit is set to: "<<int(AutoConfirmTransmit)<<std::endl;
				
			};
			*/
	//for(unsigned int i=0;i<entries.size();i++){
	//	Entries.push_back(entries[i]);
	//};
};

Attribute::Attribute(const Attribute &attribute):
	Attribute(	attribute.BioFlexBusId,
			attribute.Name, 
			attribute.ProtocolId, 
			attribute.RequestId, 
			attribute.Entries, 
			attribute.SendMessage, 
			attribute.GetIterationNumber, 
			attribute.IsTransmittable, 
			attribute.AutoConfirmTransmit){
};

Attribute& Attribute::operator=(const Attribute& attribute){
	if (this != &attribute) // protect against invalid self-assignment
		{
		BioFlexBusId=attribute.BioFlexBusId;
		Name=attribute.Name;
		ProtocolId=attribute.ProtocolId;
		RequestId=attribute.RequestId;
		Entries=attribute.Entries;
		SendMessage=attribute.SendMessage;
		GetIterationNumber=attribute.GetIterationNumber;
		IsTransmittable=attribute.IsTransmittable;
		AutoConfirmTransmit=attribute.AutoConfirmTransmit;
	}
	return *this;
}


std::string Attribute::GetName() const{
	return Name;
};

void Attribute::SetName(std::string name){
	Name=name;
}

unsigned char Attribute::GetProtocolId() const{
	return ProtocolId;
}

void Attribute::SetProtocolId(unsigned char protocolId){
	ProtocolId=protocolId;
}

unsigned char Attribute::GetCommandId() const{
	return RequestId;
};

void Attribute::SetCommandId(unsigned char commandId){
	RequestId=commandId;
}

const std::vector<Entry> Attribute::GetEntries() const{
	return Entries;
};

unsigned char Attribute::GetBioFlexBusId(){
	return BioFlexBusId;
}

void Attribute::SetBioFlexBusId(unsigned char bioFlexBusId){
	BioFlexBusId=bioFlexBusId;
}

void Attribute::SetIterationNumberHandle(std::function< unsigned long int ()> getIterationNumber){
	GetIterationNumber=getIterationNumber;
}

void Attribute::SetSendMessageHandle(AttributeUtilities::sendMessageHandle_t sendMessage){
	SendMessage=sendMessage;
}

void Attribute::SetNotifyOfErrorHandle(std::function<void()> notifyOfError){
	NotifyOfError=notifyOfError;
}

boost::shared_ptr<BfbMessage> Attribute::CreateRequest(){
	auto message=boost::make_shared<BfbMessage>(BioFlexBusId, 2, true, false, ProtocolId, RequestId);
	return message;
};


void Attribute::HandleRequestReply(boost::shared_ptr<const BfbMessage > message){
	if(message==nullptr){
		throw std::out_of_range("The requested data could not be acquired");
	};
	if(message->GetError()){
		NotifyOfError();
	}
	if(message->GetSource()!=BioFlexBusId){
		return;
	};
	auto const payload=message->GetPayload();
	unsigned long index=0;
	variableTypeList value;
	for(unsigned long int i=0; i<Entries.size(); i++){
		if(Entries[i].HostType!=STRING){
			unsigned char numOfBytes=0;
			bool isUnsigned=false;
			switch(Entries[i].ClientType){
				case BOOL:
					isUnsigned=true;
					numOfBytes=1;
					break;
				case UINT8:
					isUnsigned=true;
				case INT8:
					numOfBytes=1;
					break;
				case UINT16:
					isUnsigned=true;
				case INT16:
					numOfBytes=2;
					break;
				case UINT32:
					isUnsigned=true;
				case INT32:
					numOfBytes=4;
					break;
				case UINT64:
					isUnsigned=true;
				case INT64:
					numOfBytes=8;
					break;
				case STRING:
				case FLOAT:
					break;
			};
			unsigned long long tempLongLong=0;
			for(unsigned int j=0;j<numOfBytes;j++){
				if(index==payload.size()){
					BfbFunctions::printMessage(message);
					std::cout<<"numOfBytes: "<<numOfBytes<<std::endl;
					throw std::out_of_range("The received message did not have the required length (it was too short)!");
				};
				tempLongLong+=((unsigned long long)payload[index])<<(8*j);
				index++;
			};
			double tempDouble=tempLongLong;
			if(!isUnsigned && tempDouble>pow(2,(numOfBytes*8)-1)-1){
				tempDouble-=pow(2,(numOfBytes*8));
			};
			double clow=Entries[i].ClientLower;
			double cupp=Entries[i].ClientUpper;
			double hlow=Entries[i].HostLower;
			double hupp=Entries[i].HostUpper;
			
			double a=(hupp-hlow)/(cupp-clow);
			double b=(clow*hupp-cupp*hlow)/(clow-cupp);
			tempDouble*=a;
			tempDouble+=b;
			
			value.push_back(tempDouble);
		}else{
			
		};
	};
	boost::lock_guard<boost::mutex> lock(*SingleAccessMutex);
	Value=value;
	State=UpToDate;
	LastUpdateIteration=GetIterationNumber();
	StateChangeNotification->notify_all();
};

double clip(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}

boost::shared_ptr<BfbMessage> Attribute::CreateTransmit(variableTypeList value){
	if(value.size()!=Entries.size()){
		throw std::out_of_range("The number of passed parameters does not fit to the number of expected parameters.");
	};
	// Test whether the types of the passed parameters fit the types of the expected parameters. Basically it is only tested whether they are both numerical or string.
	static const auto stringType=typeid(std::string("Hello World")).name(); // This is used since typeid may give different results on different operating systems.
	auto itV=value.begin();
	auto itE=Entries.begin();
	for(;itV!=value.end() && itE!=Entries.end(); ++itV, ++itE){
		bool vIsString=(itV->type().name()==stringType);
		bool eIsString=(itE->HostType==STRING);
		if(vIsString!=eIsString){
			throw std::invalid_argument("The type of at least one of the passed parameter does not match the expected type.");
		};
	};
	
	auto message=boost::make_shared<BfbMessage>();
	message->SetDestination(BioFlexBusId);
	message->SetSource(2);
	if(AutoConfirmTransmit){
		message->SetBusAllocationFlag(true);
	}
	message->SetProtocol(ProtocolId);
	message->SetCommand(RequestId+2);
	
	std::vector<unsigned char> payload;
	payload.clear();
	payload.reserve(Entries.size()*4);
	for(unsigned long int i=0; i<Entries.size(); i++){
		if(Entries[i].HostType!=STRING && Entries[i].ClientType!=STRING){
			double tempDouble=boost::get<double>(value[i]);
			double clow=Entries[i].ClientLower;
			double cupp=Entries[i].ClientUpper;
			double hlow=Entries[i].HostLower;
			double hupp=Entries[i].HostUpper;
			
			double a=(cupp-clow)/(hupp-hlow);
			double b=(clow*hupp-cupp*hlow)/(hupp-hlow);
			tempDouble*=a;
			tempDouble+=b;
			if(Entries[i].Limit){
				tempDouble=clip(tempDouble, clow, cupp);
			};
			unsigned char numOfBytes=0;
			bool isUnsigned=false;
			switch(Entries[i].ClientType){
				case BOOL:
					isUnsigned=true;
					numOfBytes=1;
					break;
				case UINT8:
					isUnsigned=true;
				case INT8:
					numOfBytes=1;
					break;
				case UINT16:
					isUnsigned=true;
				case INT16:
					numOfBytes=2;
					break;
				case UINT32:
					isUnsigned=true;
				case INT32:
					numOfBytes=4;
					break;
				case UINT64:
					isUnsigned=true;
				case INT64:
					numOfBytes=8;
					break;
				case STRING:
				case FLOAT:
					break;
			};
			if(isUnsigned){
				tempDouble=clip(tempDouble, 0, pow(2,8*numOfBytes)-1);
			}else{
				tempDouble=clip(tempDouble, -pow(2,8*numOfBytes)/2, pow(2,8*numOfBytes)/2-1);
			};
			signed long long tempLongLong=round(tempDouble);
			for(unsigned int j=0;j<numOfBytes;j++){
				payload.push_back((tempLongLong>>(8*j)&255));
			};
		}else if(Entries[i].HostType==STRING && Entries[i].ClientType==STRING){
			std::string tempString=boost::get<std::string>(value[i]);
			std::vector<unsigned char> tempVec;
			tempVec.resize(tempString.size());
			std::copy(tempString.begin(), tempString.end(), tempVec.data());
			payload.insert(payload.end(), tempVec.begin(), tempVec.end());
		}else{
			throw std::invalid_argument("It is not allowed to convert something from string to a numerical value or vice versa.");
		}
	};
	message->SetPayload(payload);
	return message;
};


void Attribute::HandleTransmitReply(boost::shared_ptr<const BfbMessage > message){
	if(message->GetError()){
		NotifyOfError();
	}
	//std::cout<<"Received Confirmation. Waiting for lock"<<std::endl;
	boost::lock_guard<boost::mutex> lock(*SingleAccessMutex);
	State=OutOfDate;
	StateChangeNotification->notify_all();
	//std::cout<<"Done withe the handling."<<std::endl;
}

void Attribute::UpdateValue(){
	const std::function<void (boost::shared_ptr<const BfbMessage> )> tempHandleRequestReply=[this](boost::shared_ptr<const BfbMessage> message)->void{
					return HandleRequestReply(message);
				};
	//auto message=CreateRequest();
	auto message=Request;
	{
		boost::unique_lock<boost::mutex> lock(*SingleAccessMutex);
		while(State==WaitingForTransmitReply || State==WaitingForRequestReply){
			StateChangeNotification->wait(lock);
		};
		State=WaitingForRequestReply;
	}
	SendMessage(message, tempHandleRequestReply);
}

void Attribute::UpdateValueIfTooOld(){
	//std::cout<<"updating too old " << Name<< std::endl;
	boost::unique_lock<boost::mutex> lock(*SingleAccessMutex);
	if( (State==UpToDate && GetIterationNumber()-MaxAge<=LastUpdateIteration) || State==WaitingForRequestReply){
		//std::cout<<"No need to cache "<<Name<<"for client with id "<<std::hex<<int(BioFlexBusId)<<std::dec<<std::endl;
		//std::cout<<"State: "<< tempStates[State]<<std::endl;
		return; // do nothing.
	}else{
		lock.unlock();
		//std::cout<<"Caching "<<Name<<" for client with id 0x"<<std::hex<<int(BioFlexBusId)<<std::dec<<std::endl;
		UpdateValue();
	};
};

void Attribute::UpdateValueIfTooOldAndWait(){
	GetValue();
};


variableTypeList Attribute::GetValue(bool getNewValue){
	boost::unique_lock<boost::mutex> lock(*SingleAccessMutex);
	if(State==UpToDate && GetIterationNumber()-MaxAge<=LastUpdateIteration && !getNewValue){
		return Value;
	}else{
		if(State!=WaitingForRequestReply){
			lock.unlock();
			UpdateValue();
			lock.lock();
		};
		while(State!=UpToDate){
			//std::cout<<"Attribute '"<< Name << "' from ID 0x"<<std::hex<< int(BioFlexBusId) <<" is out of date."<<std::endl;
			StateChangeNotification->wait(lock);
		};
		return Value;
	};
};

void Attribute::SetValue(variableTypeList value){
	SetValue(value, AutoConfirmTransmit);
};

void Attribute::SetValue(variableTypeList value, bool blocking){
	if(!IsTransmittable){
		throw std::invalid_argument("This attribute ('" + Name + "') cannot be set.");
	};
	const std::function<void (boost::shared_ptr<const BfbMessage>)> tempHandleTransmitReply=
			  [this](boost::shared_ptr<const BfbMessage> message)->void{
					   HandleTransmitReply(message);
	};
	auto tempMessage=CreateTransmit(value);
	tempMessage->SetBusAllocationFlag(blocking);
	
	boost::unique_lock<boost::mutex> lock(*SingleAccessMutex);
	
	while(State==WaitingForRequestReply || State==WaitingForTransmitReply){
		StateChangeNotification->wait(lock);
	};
	SendMessage(tempMessage, tempHandleTransmitReply);
	
	if(blocking){
		State=WaitingForTransmitReply;
		while(State==WaitingForTransmitReply){
			StateChangeNotification->wait(lock);
		};
	}
	State=OutOfDate;
};
