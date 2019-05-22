// STL includes
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

// Boost includes
#include <boost/assign.hpp>
#include <boost/date_time.hpp>
#include <boost/thread.hpp>
#include <boost/thread/pthread/mutex.hpp>

// Boost includes
#include "BfbMessage.hpp"

using namespace BfbConstants;
using namespace BfbFunctions;

namespace BfbFunctions{
	bool isValidShortPacket(std::vector<unsigned char> rawData){
		try{
			if( (rawData.at(flagsPos) & sizeFlag_bm)==0x00 && rawData.at(shortCrcPos)==shortCrcDummy  && rawData.size()==shortLength){
				return true;
			}
		}catch(const std::out_of_range& oor){
			//std::cout<<oor.what()<<std::endl; //This can be used to inform the user that a too small vector was supplied. Here it is not used since a too small vector is not a valid long message header and therefore the function can safely ignore it.
		};
		return false;
	}
	
	bool isValidLongPacketHeader(std::vector<unsigned char> rawData){
		try{
			if( (rawData.at(flagsPos) & sizeFlag_bm) == longPacketFlag_bm && rawData.size()<=rawData.at(longLengthPos) && rawData.at(longLengthPos)<=longMessageMaxLength && rawData.at(longHeaderCrcPos)==longHeaderCrcDummy){
				return true;
			}
		}catch(const std::out_of_range& oor){
			//std::cout<<oor.what()<<std::endl; //This can be used to inform the user that a too small vector was supplied. Here it is not used since a too small vector is not a valid long message header and therefore the function can safely ignore it.
		};
		return false;
	}
	
	bool isValidUltraLongPacketHeader(std::vector<unsigned char> rawData){
		try{
			unsigned long expectedLength=0;
			for(unsigned int i=0;i<4;i++){
				expectedLength+=pow(2,8*i)*static_cast<unsigned long>(rawData.at(ultraLongLengthStart+i));
			};
			if( (rawData.at(flagsPos) & sizeFlag_bm) == ultraLongPacketFlag_bm && rawData.size()<=expectedLength && rawData.at(ultraLongHeaderCrcPos)==ultraLongHeaderCrcDummy){
				return true;
			}
		}catch(const std::out_of_range& oor){
			//std::cout<<oor.what()<<std::endl; //This can be used to inform the user that a too small vector was supplied. Here it is not used since a too small vector is not a valid long message header and therefore the function can safely ignore it.
		};
		return false;
	}

	bool isValidLongPacket(std::vector<unsigned char> rawData){
		try{
			if( isValidLongPacketHeader(rawData) && rawData.size()==rawData.at(longLengthPos) && rawData.at(rawData.size()+longPayloadCrcStart)==longPayloadCrcDummy1 && rawData.at(rawData.size()+longPayloadCrcStart+1)==longPayloadCrcDummy2){
				return true;
			}
		}catch(const std::out_of_range& oor){
			//std::cout<<oor.what()<<std::endl; //This can be used to inform the user that a too small vector was supplied. Here it is not used since a too small vector is not a valid long message header and therefore the function can safely ignore it.
		};
		return false;
	};

	bool isValidUltraLongPacket(std::vector<unsigned char> rawData){
		try{
			unsigned long expectedLength=0;
			for(unsigned int i=0;i<4;i++){
				expectedLength+=pow(2,8*i)*static_cast<unsigned long>(rawData.at(ultraLongLengthStart+i));
			};
			if( isValidUltraLongPacketHeader(rawData) && rawData.size()==expectedLength && rawData.at(rawData.size()+ultraLongPayloadCrcStart)==ultraLongPayloadCrcDummy1 && rawData.at(rawData.size()+ultraLongPayloadCrcStart+1)==ultraLongPayloadCrcDummy2){
				return true;
			}
		}catch(const std::out_of_range& oor){
			//std::cout<<oor.what()<<std::endl; //This can be used to inform the user that a too small vector was supplied. Here it is not used since a too small vector is not a valid long message header and therefore the function can safely ignore it.
		};
		return false;
	};
	
	unsigned long numOfMissingBytes(std::vector<unsigned char> rawData){
		try{
			if(rawData.size()<8){
				return shortLength-rawData.size(); 	// Return the number of bytes that are needed to complete a short message. 
									// If the supplied bytes are part of a long message, the correct number of missing bytes will be returned when this function is executed with the vector holding the eight bytes. 
			}else if(isValidShortPacket(rawData)){
				return 0;
			}else if(isValidLongPacketHeader(rawData)){
				return rawData.at(longLengthPos)-rawData.size();
			}else if(isValidUltraLongPacketHeader(rawData)){
				unsigned long expectedLength=0;
				for(unsigned int i=0;i<4;i++){
					expectedLength+=pow(2,8*i)*static_cast<unsigned long>(rawData.at(ultraLongLengthStart+i));
				};
				return expectedLength-rawData.size();
			};
		}catch(const std::out_of_range& oor){
		};
		throw std::invalid_argument("The supplied vector contains neither a short/long/ultralong message nor a fraction of one of them.");
	}
	
	static boost::mutex singlePrintMutex;
	void printMessage(const BfbMessage &message, std::string foreword, signed long int maxPayloadPrintout){
		boost::unique_lock<boost::mutex> lock(singlePrintMutex);
		if(foreword!=std::string("")){
			std::cout<<foreword<<std::endl;
		};
		unsigned char textWidth=17;
		unsigned char numberWidth=5;
		std::cout<<std::left<<std::setw(textWidth)<<	"Destination-ID: "	<< std::right<< std::dec << std::setw(numberWidth)<<int(			message.GetDestination()	)<<" ( "<< std::hex<< std::showbase<<uint(message.GetDestination())<<" )"<<std::endl;
		std::cout<<std::left<<std::setw(textWidth)<<	"Source-ID: "		<< std::right<< std::dec << std::setw(numberWidth)<<int(			message.GetSource()		)<<" ( "<< std::hex<< std::showbase<<uint(message.GetSource())<<" )"<<std::endl;
		std::cout<<std::left<<std::setw(textWidth)<<	"Bus Allocation? "	<< std::right<< std::dec << std::setw(numberWidth)<< std::boolalpha<<		message.GetBusAllocation()	<<std::endl;
		std::cout<<std::left<<std::setw(textWidth)<<	"Error? "		<< std::right<< std::dec << std::setw(numberWidth)<< std::boolalpha<<		message.GetError()		<<std::endl;
		std::cout<<std::left<<std::setw(textWidth)<<	"Protocol-ID: "		<< std::right<< std::dec << std::setw(numberWidth)<<int(			message.GetProtocol()		)<<" ( "<< std::hex<< std::showbase<<uint(message.GetProtocol())<<" )"<<std::endl;
		std::cout<<std::left<<std::setw(textWidth)<<	"Command-ID: "		<< std::right<< std::dec << std::setw(numberWidth)<<int(			message.GetCommand()		)<<" ( "<< std::hex<< std::showbase<<uint(message.GetCommand())<<" )"<<std::endl;
		auto tempPayload=message.GetPayload();
		if(maxPayloadPrintout>0 && tempPayload.size()>static_cast<unsigned long int>(maxPayloadPrintout)){
			std::cout<<"The payload of this message is "<<int(tempPayload.size())<< " Bytes long. Only "<< maxPayloadPrintout<< " Bytes will be printed!"<<std::endl;
			tempPayload.resize(static_cast<unsigned long int>(maxPayloadPrintout));
		};
		std::cout<<std::left<<std::setw(textWidth)<<	"Payload: "		<< std::right<< std::dec << std::setw(numberWidth);
		
		for(int i=0;i<int(tempPayload.size())-1;i++){
			std::cout<< std::setw(numberWidth)<<std::dec<<						   int(			tempPayload.at(i)			)<<", ";
			if(i%10==9){
				std::cout<<std::endl<<std::setw(textWidth)<<" ";
			};
		};
		if(tempPayload.size()>0){
			std::cout<< std::setw(numberWidth)<<std::dec<<						   int(			tempPayload.back()			)<<std::endl;
		}
		std::cout<<std::right<<std::setw(textWidth)<<"( ";
		for(int i=0;i<int(tempPayload.size())-1;i++){
			std::cout<< std::setw(numberWidth)<<std::hex<< std::showbase<<						   int(			tempPayload.at(i)			)<<", ";
			if(i%10==9){
				std::cout<<std::endl<<std::setw(textWidth)<<" ";
			};
		};
		if(tempPayload.size()>0){
			std::cout<< std::setw(numberWidth)<<std::hex<< std::showbase<<						   int(			tempPayload.back()			)<<" )"<<std::endl;
		};
		std::string comment=message.GetComment();
		if(comment!=""){
			std::cout<<std::left<<std::setw(textWidth)<<	"Comment: "	<< comment << std::endl;
		}
		std::cout<<std::dec<<std::endl;
	};
	
	void printMessage(boost::shared_ptr<const BfbMessage> message, std::string foreword, signed long int maxPayloadPrintout){
		printMessage(*message, foreword, maxPayloadPrintout);
	}
	
	double clip(double input, double lower, double upper) {
		return std::max(lower, std::min(input, upper));
	}
	
	std::vector<unsigned char> convertDoubleToBytes(const double input, const unsigned char numOfBits, const bool isSigned){
		if(numOfBits%8!=0){
			throw std::invalid_argument("The number of bits must a multiple of 8.");
		};
		double tempDouble;
		if(isSigned){
			tempDouble=clip(input, -pow(2,numOfBits)/2, pow(2,numOfBits)/2-1);
		}else{
			tempDouble=clip(input, 0, pow(2,numOfBits)-1);
		};
		signed long int tempLong=round(tempDouble);
		std::vector<unsigned char> output;
		output.reserve(numOfBits/8);
		for(unsigned int j=0;j<(numOfBits/8);j++){
			output.push_back(((tempLong>>(8*j))&255));
		};
		return output;
	}
	
	std::vector<unsigned char> convertDoublesToBytes(std::vector<double> input, unsigned char numOfBits, bool isSigned){
		std::vector<unsigned char> output;
		output.reserve(input.size()*(numOfBits/8));
		for(auto it=input.begin(); it!=input.end(); it++){
			auto temp=convertDoubleToBytes(*it, numOfBits, isSigned);
			output.insert(output.end(), temp.begin(), temp.end());
		};
		return output;
	};
	
	
	

}


BfbMessage::BfbMessage():
	Payload(std::vector<unsigned char>(0)){//,
};

BfbMessage::BfbMessage(unsigned char* rawdata, uint16_t rawdataLength):
	BfbMessage(){
	this->SetRawData(rawdata, rawdataLength);
	//	throw std::invalid_argument( "The received parameter contains invalid data that cannot be used to create a standard BioFlex bus message." );
}

BfbMessage::BfbMessage(std::vector<unsigned char> rawData):
	BfbMessage(){
	this->SetRawData(rawData);
	//	throw std::invalid_argument( "The received parameter contains invalid data that cannot be used to create a standard BioFlex bus message." );
}


BfbMessage::BfbMessage(unsigned char destination, unsigned char source, bool busAllocation, bool error, unsigned char protocol, unsigned char command, std::vector< unsigned char > payload, std::string comment):
	Destination(destination),
	Source(source),
	BusAllocation(busAllocation),
	Error(error),
	Protocol(protocol),
	Command(command),
	Payload(payload),
	Comment(comment){
};

BfbMessage::BfbMessage(const BfbMessage &message):
	Destination(message.GetDestination()),
	Source(message.GetSource()),
	BusAllocation(message.GetBusAllocation()),
	Error(message.GetError()),
	Protocol(message.GetProtocol()),
	Command(message.GetCommand()),
	Payload(message.GetPayload()),
	Comment(message.GetComment()){//,
	//MaximumNumberOfTransmissions(message.MaximumNumberOfTransmissions),
	//NumberOfTransmissions(message.NumberOfTransmissions),
	//TimeOfLastTransmission(message.TimeOfLastTransmission){
}


BfbMessage::~BfbMessage(){};

unsigned char BfbMessage::GetDestination() const {
	return this->Destination;
};

unsigned char BfbMessage::GetSource() const {
	return this->Source;
};

unsigned char BfbMessage::GetProtocol() const {
	return this->Protocol;
};

unsigned char BfbMessage::GetCommand() const {
	return this->Command;
};

bool 	BfbMessage::GetBusAllocation() const {
	return this->BusAllocation;
};

bool 	BfbMessage::GetError() const {
	return this->Error;
};

std::vector<unsigned char> BfbMessage::GetPayload() const {
	return this->Payload;
};

std::vector<unsigned char> BfbMessage::GetRawData() const {
	std::vector<unsigned char> rawData(8,0);
	if(Payload.size()<=2){ //Short format
		rawData.resize(8);
		rawData[shortProtocolPos]=this->Protocol;
		rawData[shortCommandPos]=this->Command;
		if(BusAllocation){
			rawData[flagsPos]=busAllocationFlag_bm;
		};
		for(unsigned int i=0;i<this->Payload.size();i++){
			rawData[i+shortPayloadStart]=this->Payload.at(i);
		};
		for(unsigned int i=this->Payload.size();i<2;i++){
			rawData[i+shortPayloadStart]=0;
		};
		rawData[shortCrcPos]=shortCrcDummy;
	}else if(Payload.size()<=(longMessageMaxLength-longMessageOverhead)){ //long format
		rawData.resize(this->Payload.size()+longMessageOverhead);
		rawData[flagsPos]|=longPacketFlag_bm;
		if(BusAllocation){
			rawData[flagsPos]|=busAllocationFlag_bm;
		};
		rawData[longLengthPos]=this->Payload.size()+longMessageOverhead;
		rawData[longHeaderCrcPos]=longHeaderCrcDummy;
		rawData[longProtocolPos]=this->Protocol;
		rawData[longCommandPos]=this->Command;
		for(unsigned int i=0;i<this->Payload.size();i++){
			rawData[i+longPayloadStart]=Payload.at(i);
		};
		rawData[rawData.size()+longPayloadCrcStart]=longPayloadCrcDummy1;
		//std::cout<<"Help"<<std::endl;
		rawData[rawData.size()+longPayloadCrcStart+1]=longPayloadCrcDummy2;
	}else if(Payload.size()<=ultraLongMessageMaxLength-ultraLongMessageOverhead){
		rawData.resize(this->Payload.size()+ultraLongMessageOverhead);
		rawData[flagsPos]|=ultraLongPacketFlag_bm;
		if(BusAllocation){
			rawData[flagsPos]|=busAllocationFlag_bm;
		};
		for(unsigned int i=0; i<4; i++){
			rawData[ultraLongLengthStart+i]=((this->Payload.size()+ultraLongMessageOverhead)>>(8*i)) & 0xFF;
		}
		rawData[ultraLongHeaderCrcPos]=ultraLongHeaderCrcDummy;
		rawData[ultraLongProtocolPos]=this->Protocol;
		rawData[ultraLongCommandPos]=this->Command;
		for(unsigned long int i=0;i<=this->Payload.size();i++){
			rawData[i+ultraLongPayloadStart]=this->Payload[i];
		};
		rawData[rawData.size()+ultraLongPayloadCrcStart]=ultraLongPayloadCrcDummy1;
		rawData[rawData.size()+ultraLongPayloadCrcStart+1]=ultraLongPayloadCrcDummy2;
	};
	rawData[destinationPos]=this->Destination;
	rawData[sourcePos]=this->Source;
	if(this->BusAllocation){
		rawData[flagsPos]|=busAllocationFlag_bm;
	};
	if(this->Error){
		rawData[flagsPos]|=errorFlag;
	};
	return rawData;
}

std::string BfbMessage::GetComment() const{ return Comment;};


void BfbMessage::SetDestination(unsigned char new_destination){
	this->Destination=new_destination;
}

void BfbMessage::SetSource(unsigned char newSource){
	this->Source=newSource;
}

void BfbMessage::SetProtocol(unsigned char newProtocol){
	this->Protocol=newProtocol;
};

void BfbMessage::SetCommand(unsigned char newCommand){
	this->Command=newCommand;
};

void BfbMessage::SetBusAllocationFlag(bool newBusAllocation){
	this->BusAllocation=newBusAllocation;
};

void BfbMessage::SetErrorFlag(bool newError){
	this->Error=newError;
};

void BfbMessage::SetPayload(std::vector<unsigned char>  newPayload){
	this->Payload=newPayload;
};

void BfbMessage::SetRawData(unsigned char* rawData, uint16_t rawDataLength){
	std::vector<unsigned char> tempVec;
	tempVec.assign(&rawData[0],&rawData[0]+rawDataLength);
	return this->SetRawData(tempVec);
}

void BfbMessage::SetRawData(std::vector<unsigned char>  newRawData){
	if(isValidShortPacket(newRawData)){
		this->Protocol=newRawData[shortProtocolPos];
		this->Command=newRawData[shortCommandPos];
		this->Payload.resize(2);
		this->Payload[0]=newRawData[shortPayloadStart+0];
		this->Payload[1]=newRawData[shortPayloadStart+1];
	}else if(isValidLongPacket(newRawData)){
		this->Protocol=newRawData[longProtocolPos];
		this->Command=newRawData[longCommandPos];
		this->Payload.resize(newRawData.size()-longMessageOverhead);
		for(unsigned int i=0;i<newRawData.size()-longMessageOverhead;i++){
			this->Payload[i]=newRawData[longPayloadStart+i];
		};
	}else if(isValidUltraLongPacket(newRawData)){
		this->Protocol=newRawData[ultraLongProtocolPos];
		this->Command=newRawData[ultraLongCommandPos];
		this->Payload.resize(newRawData.size()-ultraLongMessageOverhead);
		for(unsigned long i=0;i<newRawData.size()-ultraLongMessageOverhead;i++){
			this->Payload[i]=newRawData[ultraLongPayloadStart+i];
		};
	}else{
		throw(std::invalid_argument("The passed raw data does not contain a valid message."));
	};
	this->Destination=newRawData[destinationPos];
	this->Source=newRawData[sourcePos];
	this->BusAllocation=bool(newRawData[flagsPos] & busAllocationFlag_bm);
	this->Error=bool(newRawData[flagsPos] & errorFlag);
};	

void BfbMessage::SetComment(std::string comment){
	Comment=comment;
}
