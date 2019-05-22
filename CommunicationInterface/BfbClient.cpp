#include "BfbClient.hpp"
#include "Attribute.hpp"
#include <BfbMessage.hpp>
#include <list>
#include <vector>
#include <boost/assign.hpp>
#include <string>
#include <boost/variant.hpp>
#include <boost/make_shared.hpp>


BfbClient::BfbClient(unsigned char bioFlexBusId, AttributeUtilities::sendMessageHandle_t sendMessageHandle, std::function<unsigned long int()> getIterationNumberHandle, std::vector<Attribute> attributes):
		BioFlexBusId(bioFlexBusId),
		FatalError(false),
		Attributes(std::map<std::string, boost::shared_ptr<Attribute>>()),
		SendMessageHandle(sendMessageHandle),
		GetIterationNumberHandle(getIterationNumberHandle){
	for(auto it=attributes.begin(); it!=attributes.end(); it++){
		AddAttribute(*it);
		//std::cout<<"Added attribute "<<it->GetName()<<" to BfbClient with ID " << int(bioFlexBusId)<<std::endl;
	};
};

void BfbClient::UpdateValue(std::string attributeName){
	Attributes.at(attributeName)->UpdateValue();
};

void BfbClient::UpdateValueIfTooOld(std::string attributeName){
	Attributes.at(attributeName)->UpdateValueIfTooOld();
};

void BfbClient::UpdateValueIfTooOldAndWait(std::string attributeName){
	Attributes.at(attributeName)->UpdateValueIfTooOld();
};

AttributeUtilities::variableTypeList BfbClient::GetValue(std::string attributeName){
	try{
		return Attributes.at(attributeName)->GetValue();
	}catch(const std::out_of_range& error){
		throw std::out_of_range("You tried to get the value for the attribute '"+attributeName+"'. This attribute is not known to the client. Maybe you forgot to specify the corresponding protocol when creating the client? Or the protocol definition isn't updated yet? Or you mistyped the name?");
	};
};

void BfbClient::SetValue(std::string attributeName, AttributeUtilities::variableTypeList value){
	try{
		return Attributes.at(attributeName)->SetValue(value);
	}catch(const std::out_of_range& error){
		throw std::out_of_range("You tried to set the value for the attribute '"+attributeName+"'. This attribute is not known to the client. Maybe you forgot to specify the corresponding protocol when creating the client? Or the protocol definition isn't updated yet? Or you mistyped the name?");
	};
};


void BfbClient::SetValue(std::string attributeName, AttributeUtilities::variableTypeList value, bool blocking){
	try{
		return Attributes.at(attributeName)->SetValue(value, blocking);
	}catch(const std::out_of_range& error){
		throw std::out_of_range("You tried to set the value for the attribute '"+attributeName+"'. This attribute is not known to the client. Maybe you forgot to specify the corresponding protocol when creating the client? Or the protocol definition isn't updated yet? Or you mistyped the name?");
	};
};
/*
void BfbClient::SetValue(std::string attributeName, double value, bool blocking){
	AttributeUtilities::variableTypeList tempList;
	tempList.push_back(value);
	SetValue(attributeName, tempList, blocking);
}

void BfbClient::SetValue(std::string attributeName, long signed int value, bool blocking){
	AttributeUtilities::variableTypeList tempList;
	tempList.push_back(value);
	SetValue(attributeName, tempList, blocking);
}
*/

void BfbClient::AddAttribute(const Attribute attribute){
	if(Attributes.count(attribute.GetName())!=0){
		std::invalid_argument("An argument with the name " +  attribute.GetName() + " has been defined multiple times! Names must be unique.");
	};
	Attributes[attribute.GetName()]=boost::make_shared<Attribute>(attribute);
	Attributes[attribute.GetName()]->SetNotifyOfErrorHandle(boost::bind(&BfbClient::NotifyOfError,this));
}

void BfbClient::AddAttribute(std::string name, unsigned char protocolId, unsigned char commandId, const std::vector<AttributeUtilities::Entry> entries, const bool isTransmittable, const bool confirmTransmit){
	if(Attributes.count(name)!=0){
		std::invalid_argument("An argument with the name " +  name + " has been defined multiple times! Names must be unique.");
	};
	Attributes[name]=boost::make_shared<Attribute>(	BioFlexBusId,
							name,
							protocolId, 
							commandId, 
							entries, 
							SendMessageHandle, 
							GetIterationNumberHandle, 
							isTransmittable, 
							confirmTransmit);
	Attributes[name]->SetNotifyOfErrorHandle(boost::bind(&BfbClient::NotifyOfError,this));
}

void BfbClient::NotifyOfError(){
	FatalError=true;
}
bool BfbClient::GetErrorState(){
	return FatalError;
}

void BfbClient::ClearErrorState(){
	FatalError=false;
}

unsigned char BfbClient::GetBioFlexBusId(){
	return BioFlexBusId;
}

