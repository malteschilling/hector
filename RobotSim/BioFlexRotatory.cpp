// STL includes
#include <iostream>
#include <math.h>
#include <string>

// Boost includes
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/shared_ptr.hpp>

// Own header files
#include "BfbProtocolIds.hpp"
#include "BioFlexRotatory.hpp"
#include "Bodies.hpp"
#include "Joints.hpp"
#include "OdeDrawstuff.hpp"

static dReal PI=boost::math::constants::pi<dReal>();

BioFlexRotatory::BioFlexRotatory(	dWorldID worldId,
					unsigned char bioFlexBusId,
					dRealVector3 anchor,
					dRealVector3 axis,
					boost::weak_ptr<Bodies::BaseBody> bodyInput,
					boost::weak_ptr<Bodies::BaseBody> bodyOutput,
					dReal initialOutputAngle,
					dReal springConstant,
					dReal dampingConstant): 
						Joints::Hinge( 	worldId,
								"BioFlexRotatory#" + boost::lexical_cast<std::string>(bioFlexBusId),
								anchor,
								axis,
								bodyInput,
								bodyOutput,
								initialOutputAngle), 
						BfbClient(bioFlexBusId),
						SpringConstant(0), 
						DampingConstant(0.4),
						MaxTorque(15),
						InputSpeed(0),
						InputAngle(initialOutputAngle),
						OutputAngle(initialOutputAngle),
						Torsion(0),
						Torque(0),
						Activation(false){
	//Activation=true; // This is just temporarily set. In the end, when the real robot should be used, 
			 // the initial state of the drives will be "deactivated". Therefore, before anything can
			 // be done with it, the drives must be activated explicitly via communication.
};

bool BioFlexRotatory::GetActivation(){return this->Activation;};
void BioFlexRotatory::SetActivation(bool activation){this->Activation=activation;};
void BioFlexRotatory::SetInputSpeed(dReal inputSpeed){this->InputSpeed=inputSpeed;};
dReal BioFlexRotatory::GetInputSpeed(){return this->InputSpeed;};
dReal BioFlexRotatory::GetOutputSpeed(){return dJointGetHingeAngleRate(JointId);};
dReal BioFlexRotatory::GetTorque(){return this->Torque;};
dReal BioFlexRotatory::GetInputAngle(){return this->InputAngle;};
dReal BioFlexRotatory::GetTorsionAngle(){return this->Torsion;};
dReal BioFlexRotatory::GetOutputAngle(){return this->OutputAngle;};
void BioFlexRotatory::SetSpringConstant(dReal springConstant){this->SpringConstant=springConstant;};
dReal BioFlexRotatory::GetSpringConstant(){return this->SpringConstant;};
void BioFlexRotatory::SetDampingConstant(dReal dampingConstant){this->DampingConstant=dampingConstant;};
dReal BioFlexRotatory::GetDampingConstant(){return this->DampingConstant;};

void BioFlexRotatory::SetInputPosition(dReal inputPos){
	this->InputAngle=inputPos;
	this->OutputAngle=inputPos;
};

void BioFlexRotatory::PostSimulationStepUpdate(double deltaT){
	Time+=deltaT;	
	
	{
		dReal tempOutputAngle=GetAngle()+2*PI*(round(OutputAngle/(2*PI)));
		if(OutputAngle-tempOutputAngle>PI){
			OutputAngle=tempOutputAngle+2*PI;
		}else if(OutputAngle-tempOutputAngle<-PI){
			OutputAngle=tempOutputAngle-2*PI;
		}else{
			OutputAngle=tempOutputAngle;
		};
	};
	dReal dampingTorque=0;
	if(Activation && !FatalError){
		InputAngle+=InputSpeed*deltaT;
		Torsion=InputAngle-OutputAngle;
		dReal outputSpeed=dJointGetHingeAngleRate(JointId);
		const dReal torsionSpeed=(InputSpeed-outputSpeed);
		dampingTorque=torsionSpeed*DampingConstant;
		dReal springTorque=0;
		if(Torsion!=0){
			springTorque=855*pow(Torsion,3)/fabs(Torsion);
		};
		Torque=springTorque+dampingTorque;
		if(Torque>MaxTorque){
			Torque=MaxTorque;
		}else if(Torque<-MaxTorque){
			Torque=-MaxTorque;
		};
	}else{
		InputAngle=OutputAngle;
		Torsion=0;
		Torque=0;
	};
	// For some strange reason, the following lines are neccessary when a kinematic/staic body is connected to the joint. Without these lines, the joint will not move at all. With these lines the joint moves the non-kinematic/static body as expected. 
	if(auto sharedBody1=Body1.lock()){
		auto bodyId1=sharedBody1->GetBodyId();
		if( bodyId1!=0){
			dBodyEnable(bodyId1);
		};
	};
	if(auto sharedBody2=Body2.lock()){
		auto bodyId2=sharedBody2->GetBodyId();
		if( bodyId2!=0){
			dBodyEnable(bodyId2);
		};
	};

	dJointAddHingeTorque(JointId, Torque);
};




std::vector<unsigned char> RadToEnc14Bit(dReal Angle){
	short tempInt=round(Angle/(2*PI)*pow(2,14));
	std::vector<unsigned char>  tempCharVector(2,0);
	tempCharVector[0]|=(tempInt&255);
	tempCharVector[1]|=(tempInt>>8);
	return tempCharVector;
};
dReal Enc14BitToRad(std::vector<unsigned char> bytes){
	short tempInt=0;
	for(unsigned int i=0;i<bytes.size() && i<2;i++){
		tempInt|=( bytes.at(i)<<(i*8) );
	};
	dReal tempDReal=tempInt;
	return tempDReal/pow(2,14)*(2*PI);
};

void BioFlexRotatory::Reset(){
	Activation=false;
	ResetState=1;
}

void BioFlexRotatory::SetMaximumTorque(dReal torque){
	MaxTorque=fabs(torque);
}


boost::shared_ptr<BfbMessage> BioFlexRotatory::ProcessMessage(boost::shared_ptr<const BfbMessage> message){
	auto reply=boost::shared_ptr<BfbMessage>(new BfbMessage(message->GetRawData()));
	reply->SetDestination(message->GetSource());
	reply->SetSource(message->GetDestination());
	reply->SetBusAllocationFlag(false);
	reply->SetErrorFlag(FatalError);
	reply->SetCommand(message->GetCommand()+1);
	reply->SetPayload(boost::assign::list_of(0)(0));
	
	bool commandFound=true;
	switch (message->GetProtocol()){
		case BfbProtocolIds::BIOFLEX_ROTATORY_1_PROT:
			switch (message->GetCommand()){
				case 0: // firmwareVersion
					reply->SetPayload(boost::assign::list_of(1));
					break;
				//1: boardVersion
				case 4: // get driveActivation
					reply->SetPayload(boost::assign::list_of(GetActivation()));
					break;
				case 6: // set driveActivation
					SetActivation(message->GetPayload().at(0));
					break;

				case 30: // get resetState
					reply->SetPayload(boost::assign::list_of(ResetState));
					break;
				case 32: {// set resetState
					auto oldResetState=ResetState;
					ResetState=message->GetPayload().at(0);
					if(oldResetState==0 && ResetState!=0){ // if the resetState changed from 0 to something else, reset the drive. 
						Reset();
					}
					break;
				};
				case 60: // get inputPosition
					reply->SetPayload(RadToEnc14Bit(GetInputAngle()));
					//std::cout<<"Sending input position."<<std::endl;
					//std::cout<<"Double: "<<GetInputAngle()<<std::endl;
					break;	
					
				case 70: // get torsion
					reply->SetPayload(RadToEnc14Bit(GetTorsionAngle()));
					break;	
					
				case 90: // get torque
					reply->SetPayload(boost::assign::list_of(GetTorque()));
					break;	
					
				case 100: // get outputPosition
					{
					//std::cout<<"Sending output position."<<std::endl;
					//std::cout<<"Double: "<<GetOutputAngle()<<std::endl;
					//auto temp=RadToEnc14Bit(GetOutputAngle());
					//std::cout<<"Vec[0]: "<<int(temp[0])<<std::endl;
					//std::cout<<"Vec[0]: "<<int(temp[1])<<std::endl;
					reply->SetPayload(RadToEnc14Bit(GetOutputAngle()));
					break;
					};
				case 112: // get inputSpeed
					reply->SetPayload(RadToEnc14Bit( (GetInputSpeed())/1000*(8192/PI) ));
					break;	
					
				case 116: // get outputSpeed
					reply->SetPayload(RadToEnc14Bit( (GetOutputSpeed())/1000*(8192/PI) ));
					break;	
				default:
					commandFound=false;
					break;
			};
			break;	
			
		case BfbProtocolIds::BIOFLEX_ROTATORY_CONTROL1_PROT:
			switch(message->GetCommand()){
				case 80: // get desiredValue_ISC
					reply->SetPayload(RadToEnc14Bit(GetInputSpeed()));
					//std::cout<<"get desiredValue_ISC: "<< RadToEnc14Bit(GetInputSpeed())<<std::endl;
					break;
				case 82: // set desiredValue_ISC
					SetInputSpeed(Enc14BitToRad(message->GetPayload()));
					//std::cout<<"set desiredValue_ISC: "<< Enc14BitToRad(Message->GetPayload())<<std::endl;
					break;
					
//				case 84: // get desiredPos
//					reply->SetPayload(RadToEnc14Bit(GetInputSpeed()));
//					//std::cout<<"get desiredValue_ISC: "<< RadToEnc14Bit(GetInputSpeed())<<std::endl;
//					break;
				case 86: // set desired Position
					SetInputPosition(Enc14BitToRad(message->GetPayload()));
					//std::cout<<"set desiredPos: "<< Enc14BitToRad(message->GetPayload())<<std::endl;
					break;	
					
				default:
					commandFound=false;
					break;	
			}
			break;
		case BfbProtocolIds::BIOFLEX_ROTATORY_ERROR_PROT:
			switch (message->GetCommand()){
				case 0: //get the fatal error state
					reply->SetPayload(boost::assign::list_of<unsigned char>(FatalError));
					break;
				case 1: // set/reset the fatal error state
					FatalError=message->GetPayload()[0];
					break;
				case 80: //get the fatal error state
					reply->SetPayload(boost::assign::list_of<unsigned char>(Tmc603aError));
					break;
				case 86: //get the fatal error state
					reply->SetPayload(boost::assign::list_of<unsigned char>(WatchdogError));
					break;
				case 92: //get the fatal error state
					reply->SetPayload(boost::assign::list_of<unsigned char>(CommunicationTimeoutError));
					break;
				case 98: //get the fatal error state
					reply->SetPayload(boost::assign::list_of<unsigned char>(TorsionMeasurementError));
					break;
				case 104: //get the fatal error state
					reply->SetPayload(boost::assign::list_of<unsigned char>(OutputAngleMeasurementError));
					break;
				case 110: //get the fatal error state
					reply->SetPayload(boost::assign::list_of<unsigned char>(OvervoltageError));
					break;
				case 116: //get the fatal error state
					reply->SetPayload(boost::assign::list_of<unsigned char>(UndervoltageError));
					break;
				case 122: //get the fatal error state
					reply->SetPayload(boost::assign::list_of<unsigned char>(RotorOvertemperatureError));
					break;
				case 128: //get the fatal error state
					reply->SetPayload(boost::assign::list_of<unsigned char>(DriverOvertemperatureError));
					break;
				case 134: //get the fatal error state
					reply->SetPayload(boost::assign::list_of<unsigned char>(InputAngleError));
					break;
				case 140: //get the fatal error state
					reply->SetPayload(boost::assign::list_of<unsigned char>(TorsionError));
					break;
				case 146: //get the fatal error state
					reply->SetPayload(boost::assign::list_of<unsigned char>(OutputAngleError));
					break;
				case 152: //get the fatal error state
					reply->SetPayload(boost::assign::list_of<unsigned char>(I2cError));
					break;
				default:
					commandFound=false;
					break;
			}
			break;
		default:
			commandFound=false;
			break;
	}
	if(commandFound){
		return reply;
	}else{
		return BfbClient::ProcessMessage(message);
	};
	
};


