// Boost includes
#include <boost/assign/list_of.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>


// Own header files
#include "DataTypes.hpp"
#include "BioFlexRotatory.hpp"
#include "BfbMessage.hpp"
#include "BfbMessageProcessor.hpp"
#include "BfbProtocolIds.hpp"
#include "Bodies.hpp"
#include "GeometryXmlParser.hpp"
#include "OdeDrawstuff.hpp"
#include "Universe.hpp"

#include <boost/algorithm/string.hpp>
#include "OdeDrawstuff.hpp"
#include <ode/ode.h> 

static std::string geometryXml=""; // The variable that holds the last geometry xml data for all tcp clients.
std::map<unsigned char, double> relTimerMap;
std::map<unsigned char, double> absTimerMap;

void SimulateUntillMinimalTimer(boost::shared_ptr<Universe> universe){
	double minAbsTime=absTimerMap.begin()->second;
	for(auto it=absTimerMap.begin(); it!=absTimerMap.end(); it++){
		if(minAbsTime>it->second){
			minAbsTime=it->second;
		};
	};
	universe->SimulateUntill(minAbsTime);
	
}

boost::shared_ptr<BfbMessage> BfbMessageProcessor::ProcessMessage(boost::shared_ptr<Universe> universe, boost::shared_ptr<const BfbMessage> message){

	const unsigned char messageDestination=message->GetDestination();
	std::vector<unsigned char> messagePayload = message->GetPayload();
	auto reply=boost::shared_ptr<BfbMessage>(new BfbMessage(message->GetRawData()));
	reply->SetDestination(message->GetSource());
	reply->SetSource(message->GetDestination());
	reply->SetCommand(message->GetCommand()+1);
	
	unsigned char lastByte=0;
	unsigned char secondLastByte=0;
	try{
		lastByte=messagePayload.at(messagePayload.size()-1);
		secondLastByte=messagePayload.at(messagePayload.size()-2);
	}catch(std::out_of_range& error){
		
	};
	reply->SetPayload(boost::assign::list_of(secondLastByte)(lastByte));
	if(messageDestination==14 && message->GetProtocol()==BfbProtocolIds::SIMSERV_1_PROT){
		switch(message->GetCommand()){
			case 0: //Setting the timer reference is currently not relevant 
			{				
			reply->SetComment("The timer reference was reset.");
				break;
			};
			case 10: 
				{
				double timerState=0;
				try{
					timerState=relTimerMap[message->GetSource()];
				}catch(std::out_of_range& err){
				};
				BfbFunctions::convertDoubleToBytes(timerState, 2, false);
				break;
				};
			case 12: // This lets the simulation run for the specified period. 
				{
					unsigned long tempInt=0;
					for(unsigned int i=0;i<messagePayload.size() && i<2;i++){
						tempInt|=( messagePayload.at(i)<<(i*8) );
					};
					relTimerMap[message->GetSource()]=dReal(tempInt)/1000;
					try{
						absTimerMap[message->GetSource()]+=dReal(tempInt)/1000;
					}catch(std::out_of_range& err){
						absTimerMap[message->GetSource()]=universe->GetTime()+dReal(tempInt)/1000;
					};
					SimulateUntillMinimalTimer(universe);
					break;
				};
			case 30: // Command ID 22 is the command for sending a new geometry xml file. This is the corresponding reply. Therefore, the last payload must be returned. 
				{
					std::vector<unsigned char> tempVec;
					tempVec.resize(geometryXml.size());
					std::copy(geometryXml.begin(), geometryXml.end(), tempVec.data());
					reply->SetPayload(tempVec);
					break;
				};
			case 32:
				{
				std::string geometryXml=std::string(reinterpret_cast<char*>(messagePayload.data()), messagePayload.size());
				universe=GeometryXmlParser::Process(geometryXml,universe);
				break;
				};
			/*case 50:
				{
				std::cout << "ASKED FOR NEW POSITION" << std::endl;
				dRealVector3 tempPosition = *(new dRealVector3());
				tempPosition.at(0) = 42;*/
				/*auto tempPosition=GetPosition();
					for(unsigned int i=0; i<3; i++){
						tempPosition[i]*=100;
					};*/
				/*reply->SetPayload(BfbFunctions::convertDoublesToBytes(boost::assign::list_of<double>(tempPosition.at(0))(tempPosition.at(1))(tempPosition.at(2)), 16, true));
				break;
				};*/
			case 60: // robotTransparency
				reply->SetPayload(boost::assign::list_of(1));
					//std::cout<<"get desiredValue_ISC: "<< RadToEnc14Bit(GetInputSpeed())<<std::endl;
				break;
			case 62:
				{
				short tempInt=0;
				std::vector<unsigned char> bytes = message->GetPayload();
				for(unsigned int i=0;i<bytes.size() && i<2;i++){
					tempInt|=( bytes.at(i)<<(i*8) );
				};
				dReal tempDReal=tempInt;
				//std::cout<<"set transp: "<< tempDReal<<std::endl;
				GeometricPrimitives::GeometricPrimitiveBase::global_alpha = tempDReal/10000;

				//reply->SetPayload(BfbFunctions::convertDoublesToBytes(boost::assign::list_of<double>(tempPosition.at(0))(tempPosition.at(1))(tempPosition.at(2)), 16, true));
				break;
				};
			case 72:
				{
				std::string testString=std::string(reinterpret_cast<char*>(messagePayload.data()), messagePayload.size());
				std::vector<std::string> strs;
				//boost::algorithm::split_regex( strs, testString, regex( "]]" ) ) ;
				boost::split(strs, testString, boost::is_any_of("]"));
				if (strs.size() > 1) {
					strs[0] = strs[0].substr(2, strs[0].size()-2);
					for(unsigned int i=1; i<6; i++) {
						strs[i] = strs[i].substr(3, strs[i].size()-3);
					};
					//for(unsigned int i=0; i<7; i++) {
					//	std::cout<<"got String: "<< strs[i] << std::endl;
					//};
				
					std::vector<std::string> strsD[6];
					//boost::algorithm::split_regex( strs, testString, regex( "]]" ) ) ;
					for(int i=0; i<6; i++){
						boost::split(strsD[i], strs[i], boost::is_any_of(","));
					};
//				for(unsigned int i=0; i<strsD.size(); i++) {
//					d = std::stod(strsD[i]);
//					std::cout<<"got double: "<< d << std::endl;
//				};
//				dRealVector3 offsetPositionCapsule = (universe->GetBody("MainBody"))->GetPosition();
//				dRealMatrix3 offsetRotationCapsule = (universe->GetBody("MainBody"))->GetRotation();
//				doubleArray3 colorCapsule=boost::assign::list_of<dReal>(1)(0)(0);
				//(universe->GetBody("MainBody"))->AddCapsule(1., 1., offsetPositionCapsule, offsetRotationCapsule, colorCapsule );
//				dsSetColorAlpha( 1., 0., 0., 1. ); 
				//double posCapsule[3] = {2., 2., 0.};
					universe->clearIntModelLineDeque();
//				boost::shared_ptr<const double *> posCapsule2 = {1., 1., 1.};
					for(unsigned int i=0; i<strsD[0].size(); i++) {
						std::vector<double> linePoints {std::stod(strsD[0][i]), std::stod(strsD[1][i]), std::stod(strsD[2][i]), 
							std::stod(strsD[3][i]), std::stod(strsD[4][i]), std::stod(strsD[5][i])};
						universe->addIntModelLine(linePoints);
//					d = std::stod(strsD[i]);
					//std::cout<<"got double: " << posCapsule2 <<  posCapsule2[1]  << std::stod(strsD[4][i]) << std::endl;
					};
				} else {
					universe->clearIntModelLineDeque();
				};
				//std::cout<<"got double: " << posCapsule2[1]  << std::stod(strsD[4][0]) << std::endl;
//				const double posCapsule[3] = {2., 2., 0.};
//				const double posCapsule2[3] = {1., 1., 1.};
				//dsDrawLine(posCapsule,posCapsule2);
				

				//universe->IntModelStartPointMap.();
				//universe->IntModelEndPointMap.();
				
				//dBodyGetRotation( universe->GetBody("MainBody") );
				//dsDrawCapsule (posCapsule, dBodyGetRotation( (universe->GetBody("MainBody"))->GetBodyId() ), 1., 1.);

				//reply->SetPayload(BfbFunctions::convertDoublesToBytes(boost::assign::list_of<double>(tempPosition.at(0))(tempPosition.at(1))(tempPosition.at(2)), 16, true));
				break;
				};
			case 76: // Set the position of the camera in the simulation:
				{
				std::string testString=std::string(reinterpret_cast<char*>(messagePayload.data()), messagePayload.size());
				std::vector<std::string> strs;
				//boost::algorithm::split_regex( strs, testString, regex( "]]" ) ) ;
				boost::split(strs, testString, boost::is_any_of("]"));
				if (strs.size() > 1) {
					strs[0] = strs[0].substr(2, strs[0].size()-2);
					strs[1] = strs[1].substr(3, strs[1].size()-3);

					std::vector<std::string> strsD[3];
					//boost::algorithm::split_regex( strs, testString, regex( "]]" ) ) ;
					for(int i=0; i<2; i++){
						boost::split(strsD[i], strs[i], boost::is_any_of(","));
					};
					
					//std::cout << strsD[0][0] << " - " << strsD[0][1] << std::endl;
					float xyz[3] = {(float) std::stod(strsD[0][0]), (float) std::stod(strsD[0][1]), (float) std::stod(strsD[0][2])};
					float hpr[3] = {(float) std::stod(strsD[1][0]), (float) std::stod(strsD[1][1]), (float) std::stod(strsD[1][2])};
					dsSetViewpoint (xyz,hpr);
					/*
					for(unsigned int i=0; i<strsD[0].size(); i++) {
						std::vector<double> linePoints {std::stod(strsD[0][i]), std::stod(strsD[1][i]), std::stod(strsD[2][i]), 
							std::stod(strsD[3][i]), std::stod(strsD[4][i]), std::stod(strsD[5][i])};
						universe->addIntModelLine(linePoints);

					};*/
				}
				break;
				};
			
		};
		return reply;
	}else if(16<=messageDestination && messageDestination<=190){
		try{
			boost::shared_ptr<BfbMessage> temp=universe->GetBfbClient(messageDestination)->ProcessMessage(message);
			return temp;
		}catch(...){
			return boost::shared_ptr<BfbMessage>();
		};
	}
	return reply;
};

