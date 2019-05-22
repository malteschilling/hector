#include <stdlib.h>
#include <array>
// Own header files
#include "BfbClient.hpp"
#include "BfbMessage.hpp"
#include "BfbProtocolIds.hpp"


BfbClient::BfbClient(unsigned char bioFlexBusId):
	BioFlexBusId(bioFlexBusId),
	FatalError(false){
		for(auto it=UniqueIdentificationData.begin(); it!=UniqueIdentificationData.end(); it++){
			(*it)=rand()%256; // Create a pseudo-random number in the range 0...255 .
		};
};

boost::shared_ptr<BfbMessage> BfbClient::ProcessMessage(boost::shared_ptr<const BfbMessage> message){
	auto reply=boost::shared_ptr<BfbMessage>(new BfbMessage(message->GetRawData()));
	reply->SetDestination(message->GetSource());
	reply->SetSource(message->GetDestination());
	reply->SetBusAllocationFlag(false);
	reply->SetErrorFlag(FatalError);
	reply->SetCommand(message->GetCommand()+1);
	reply->SetPayload(boost::assign::list_of(0)(0));
	bool commandFound=true;
	switch(message->GetProtocol()){
		case BfbProtocolIds::BIOFLEX_1_PROT:
			switch(message->GetCommand()){
				case 0x00:{
					std::vector<unsigned char> tempVector;
					for(auto it=UniqueIdentificationData.begin(); it!=UniqueIdentificationData.end(); it++){
						tempVector.push_back(*it);
					};
					reply->SetPayload(tempVector);
					break;
				}
				default:
					commandFound=false;
					break;
			};
			break;
		default:
			commandFound=false;			
			break;
	}
	if(commandFound){
		return reply;
	}else{
		return boost::shared_ptr<BfbMessage>();
	};
};
//void BioFlexBusClient::PreSimulationStepUpdate(){};
void BfbClient::PostSimulationStepUpdate(double deltaT){
	(void)deltaT;
};