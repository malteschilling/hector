// STL includes
#include <cmath>
#include <vector>

// Boost includes
#include <boost/assign.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/shared_ptr.hpp>

// Own header files
#include "BfbProtocolIds.hpp"
#include "DataTypes.hpp"
#include "OdeDrawstuff.hpp"
#include "GeometricPrimitives.hpp"
#include "PressureSensor.hpp"

PressureSensor::PressureSensor(unsigned char bioFlexBusId, boost::weak_ptr<Bodies::BaseBody> body, dRealVector3  offsetPosition, dRealMatrix3  offsetRotation):
		BfbClient(bioFlexBusId),
		Body(body){ 
			if(auto sharedBody=body.lock()){
				sharedBody->RouteCollisionFeedbacksTo(boost::bind(&PressureSensor::AddCollisionFeedback, this, _1));
			};// TODO: This should throw an error if the body cannot be locked.
}

void PressureSensor::AddCollisionFeedback(const CollisionFeedback collisionFeedback){
	CollisionFeedbacks.push_back(CollisionFeedback(collisionFeedback));
};

void PressureSensor::PostSimulationStepUpdate(double deltaT){
	Forces.clear();
	for(auto it=CollisionFeedbacks.begin(); it!=CollisionFeedbacks.end(); it++){
		Forces.push_back(it->FeedbackForce);
	};
	CollisionFeedbacks.clear();
}

dReal PressureSensor::GetHighestPressure(){
	dReal highestPressure=0;
	for(auto it=Forces.begin(); it!=Forces.end(); it++){
		dReal tempPressure=sqrt(pow(it->at(0),2)+pow(it->at(1),2)+pow(it->at(2),2));
		if(tempPressure>highestPressure){
			highestPressure=tempPressure;
		};
	};
	//std::cout<<"Highest Pressure: "<<highestPressure<<std::endl;
	return highestPressure;
}

boost::shared_ptr<BfbMessage> PressureSensor::ProcessMessage(boost::shared_ptr<const BfbMessage> message){
	auto reply=boost::shared_ptr<BfbMessage>(new BfbMessage(message->GetRawData()));
	reply->SetDestination(message->GetSource());
	reply->SetSource(message->GetDestination());
	reply->SetBusAllocationFlag(false);
	reply->SetCommand(message->GetCommand()+1);
	reply->SetPayload(boost::assign::list_of(0)(0));
	switch (message->GetProtocol()){
		case BfbProtocolIds::PRESSURE_SENSOR_PROT:
			switch (message->GetCommand()){
				case 4:{ // get max_value
					dReal tempPressure=ceil(GetHighestPressure());
					std::vector<unsigned char> tempBytes(1,0);
					short tempShort;
					if(tempPressure>=pow(2,8)){
						tempShort=pow(2,8)-1;
					}else{
						tempShort=tempPressure;
					};
					tempBytes[0]=tempShort;
					reply->SetPayload(tempBytes);
					break;
				};
				case 10:{ // get highestPressure
					dReal tempPressure=GetHighestPressure();
					std::vector<unsigned char> tempBytes(2,0);
					short tempShort;
					if(ceil(tempPressure*100)>=pow(2,15)){
						tempShort=pow(2,15)-1;
					}else if(ceil(tempPressure*100)<-pow(2,15)){
						tempShort=-pow(2,15);	
					}else{
						tempShort=ceil(tempPressure*100);
					};
					tempBytes[0]|=(tempShort&255);
					tempBytes[1]|=((tempShort>>8)&255);
					reply->SetPayload(tempBytes);
					break;
				};
		}
		break;
	}
	return reply;
};