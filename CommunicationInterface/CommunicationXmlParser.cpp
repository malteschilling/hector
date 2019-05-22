// STL includes
#include <array>
#include <fstream>
#include <stdlib.h>
#include <istream>
#include <sstream>
#include <set>
#include <limits>

// Boost includes
#include <boost/algorithm/string.hpp>
#include <boost/assign/ptr_list_of.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/shared_ptr.hpp>

// External library files
#include <pugixml.hpp>

// "Header-only library files
#include "exprtk.hpp" // This library is used in order to parse the client/host lower/upper values. 

// Own header files
#include "CommunicationXmlParser.hpp"
#include "MathParser.hpp"



void print(pugi::xml_node node,unsigned int maxdepth=2, std::string indentation="")
{
	if(std::string(node.name()).size()!=0){
		std::cout<<indentation<<node.name()<<std::endl;
		if(node.attributes_begin()!=node.attributes_end()){
			std::cout<<indentation<<"    "<<"Attributes:"<<std::endl;
			for(auto it=node.attributes_begin();it!=node.attributes_end();it++){
				std::cout<<indentation<<"        "<<it->name()<<": "<<it->value()<<std::endl;
			};
		}
		if(node.begin()!=node.end()){
			for(auto it=node.begin();it!=node.end();it++){
				print(*it,maxdepth-1,indentation+"    ");
			};
		}else if(std::string(node.value()).size()!=0){
			std::cout<<indentation<<"        "<<"Value: "<<node.value()<<std::endl;
		};
	}else{
		std::cout<<indentation<<"        "<<node.value()<<std::endl;
	};
};


template <typename Type>
Type getValue(pugi::xml_node node, std::string name){
	std::string tempStr=node.child_value(name.c_str());
	if(tempStr.size()==0){
		for(auto it=node.begin();it!=node.end();it++){
			if(boost::to_lower_copy<std::string>(it->name())==boost::to_lower_copy<std::string>(name)){
				tempStr=it->value();
				break;
			}
		}
	}
	
	if(tempStr.size()==0){
		tempStr=node.attribute(name.c_str()).value();
	}
	
	if(tempStr.size()==0){
		for(auto it=node.attributes_begin();it!=node.attributes_end();it++){
			if(boost::to_lower_copy<std::string>(it->name())==boost::to_lower_copy<std::string>(name)){
				tempStr=it->value();
				break;
			}
		}
	}
	
	if(tempStr.size()==0){
		throw std::invalid_argument("No value found for entry with name '"+ name + "'.");
	}
	//std::cout<<"about to convert "+ name<<std::endl;
	return boost::lexical_cast<Type>(tempStr);
};

template <>
unsigned char getValue<unsigned char>(pugi::xml_node node, std::string name){
	return boost::numeric_cast<unsigned char>(getValue<unsigned int>(node, name));
};

template <>
signed char getValue<signed char>(pugi::xml_node node, std::string name){
	return boost::numeric_cast<signed char>(getValue<signed int>(node, name));
};

template <>
bool getValue<bool>(pugi::xml_node node, std::string name){
	std::string tempStr=getValue<std::string>(node, name);
	if(boost::to_lower_copy(tempStr)=="true"){
		return true;
	}else if(boost::to_lower_copy(tempStr)=="false"){
		throw std::invalid_argument("Trying to convert an entry to bool, but it turns out it is impossible to convert '"+ tempStr + "' to bool.");
	};
	return false;
};

template <typename Type>
Type getValue(pugi::xml_node node, std::string name, Type defaultValue){
	Type tempValue;
	try{
		tempValue=getValue<Type>(node, name);
	}catch(std::invalid_argument &err){
		tempValue=defaultValue;
	};
	return tempValue;
};


Attribute ProcessPropertyNode(pugi::xml_node propertyNode){
	std::string name;
	unsigned char requestId;
	std::string documentation="";
	signed long maxAge=0;
	double timeToWaitForAnswer=std::numeric_limits< double >::quiet_NaN();
	

	name=getValue<std::string>(propertyNode, "name");
	requestId=getValue<unsigned char>(propertyNode, "requestId");
	//std::cout<<int(requestId)<<std::endl;
	bool isTransmittable=getValue<bool>(propertyNode, "transmittable", false);
	bool autoConfirmTransmit=getValue<bool>(propertyNode, "autoconfirm", false);
	maxAge=getValue<signed long int>(propertyNode, "maxage", 0);
	documentation=getValue<std::string>(propertyNode, "doc", "");
	timeToWaitForAnswer=getValue<double>(propertyNode, "timetowaitforanswer", std::numeric_limits< double >::quiet_NaN());
	
	
	std::vector<AttributeUtilities::Entry> entries;
	for(auto dataIt=propertyNode.begin();dataIt!=propertyNode.end();dataIt++){
		if(boost::to_lower_copy<std::string>(dataIt->name())=="data"){
			AttributeUtilities::DataType_t hostType;
			AttributeUtilities::DataType_t clientType;
			double clientLower;
			double clientUpper;
			double hostLower;
			double hostUpper;
			bool limit;
			try{
				clientType=AttributeUtilities::StringDataTypeMap.at(getValue<std::string>(*dataIt, "clienttype","empty"));
				hostType=AttributeUtilities::StringDataTypeMap.at(getValue<std::string>(*dataIt, "hosttype","empty"));
			}catch(std::out_of_range& err){
				std::cout<<"In the definition of the attribute '"<<name<<"' a type was used that is not recognized."<<std::endl;
				throw;
			}
				
			clientLower=MathParser::parseDouble(getValue<std::string>(*dataIt, "clientlower","0"));
			clientUpper=MathParser::parseDouble(getValue<std::string>(*dataIt, "clientupper","1"));
			hostLower=MathParser::parseDouble(getValue<std::string>(*dataIt, "hostlower","0"));
			hostUpper=MathParser::parseDouble(getValue<std::string>(*dataIt, "hostupper","1"));
			
			limit=getValue<bool>(*dataIt, "limit", false);
			
			auto tempEntry=AttributeUtilities::Entry(	clientType,
									clientLower,
									clientUpper,
									hostType,
									hostLower,
									hostUpper,
									limit);
			entries.push_back(tempEntry);
			
		};
	};
	return Attribute(0,
			name, 
			0, 
			requestId, 
			entries, 
			[](boost::shared_ptr< const BfbMessage > message, std::function<void (boost::shared_ptr<const BfbMessage>)> callBackFunction)->void{return;},
			[](){return 0;}, 
			isTransmittable, 
			autoConfirmTransmit,
			maxAge,
			timeToWaitForAnswer);
};


CommunicationXmlParser::AttributeMap CommunicationXmlParser::Process(std::istream& stream){
	pugi::xml_document doc;
	pugi::xml_parse_result result = doc.load(stream);
	if(!result){
		throw std::invalid_argument("Could not parse the stream.");
	}
	//print(doc.first_child());
	
	
	std::list<pugi::xml_node> protocols;
	if(boost::to_lower_copy<std::string>(doc.first_child().name())=="protocol"){
		protocols.push_back(doc.first_child());
	}else if(boost::algorithm::to_lower_copy<std::string>(doc.first_child().name())=="protocollist"){
		pugi::xml_node protListNode=doc.first_child();
		for(auto it=protListNode.begin();it!=protListNode.end();it++){
			if(boost::to_lower_copy<std::string>(it->name())=="protocol"){
				protocols.push_back(*it);
			};
		};
	}else{
		stream.seekg(0);
		char hint[200];
		stream.get(hint, 200);
		throw std::invalid_argument("The passed parameter contains neither a valid path to an xml-file nor the protocol specification in xml-format as a string. The passed stream/string starts with: "+ std::string(hint));
	};
	CommunicationXmlParser::AttributeMap tempAttrMap;
	std::set<unsigned char> protIds;
	for(auto protocolIt=protocols.begin();protocolIt!=protocols.end();protocolIt++){
		CommunicationXmlParser::AttributeList tempAttrList;
		std::string protName=getValue<std::string>(*protocolIt, "name");
		unsigned char protId=getValue<unsigned char>(*protocolIt, "id");
		for(auto propertyIt=protocolIt->begin();propertyIt!=protocolIt->end();propertyIt++){
			Attribute tempAttr=ProcessPropertyNode(*propertyIt);
			tempAttr.SetProtocolId(protId);
			tempAttrList.push_back(tempAttr);
		};
		if(tempAttrMap.count(protName)==0 && protIds.count(protId)==0){
			tempAttrMap[protName]=tempAttrList;
			protIds.insert(protId);
		}else{
			throw std::invalid_argument("A protocol with the name '"+ protName + "' or the protocol-ID " + boost::lexical_cast<std::string>(int(protId)) + " already exists.");
		};
	}
	return tempAttrMap;	
};

CommunicationXmlParser::AttributeMap CommunicationXmlParser::Process(std::string xml){
	std::ifstream ifs;
	ifs.open(xml, std::ifstream::in);
	if(ifs.good()){
		return Process(ifs);
	}else{
		std::stringstream tempStream(xml);
		return Process(tempStream);
	}
};