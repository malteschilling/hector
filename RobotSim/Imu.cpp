//STL includes
#include <cmath>

//Boost includes
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/vector.hpp>


 #include <boost/numeric/ublas/vector.hpp>
 #include <boost/numeric/ublas/vector_proxy.hpp>
 #include <boost/numeric/ublas/matrix.hpp>
 #include <boost/numeric/ublas/triangular.hpp>
 #include <boost/numeric/ublas/lu.hpp>
 #include <boost/numeric/ublas/io.hpp>

// Own header files
#include "BfbClient.hpp"
#include "BfbProtocolIds.hpp"
#include "BfbMessage.hpp"
#include "HelperFunctions.hpp"
#include "Imu.hpp"


Imu::Imu(unsigned char bioFlexBusId, 
	 boost::weak_ptr<Bodies::BaseBody> body,
	 dRealVector3  offsetPosition, 
	 dRealMatrix3 offsetRotation):
		BfbClient(bioFlexBusId), 
		Body(body), 
		OffsetPosition(offsetPosition),
		OffsetRotation(offsetRotation),
		InvOffsetRotation(HelperFunctions::invertRotationMatrix(OffsetRotation)){
}

dRealVector3 Imu::GetPosition(){
	auto sharedBody=Body.lock();
	if(!sharedBody){ //TODO: This should throw an error!
		std::cout<<"The position of the IMU cannot be retrieved since the connected body does not exist anymore."<<std::endl;
	}
	dRealVector3 bodyPosition = sharedBody->GetPosition(); // the position of the body in the global coordinate system.
	dRealVector3 relImuPosition; // the position of the IMU relative to the position of the body in global coordinates.
	dBodyVectorToWorld (sharedBody->GetBodyId(), 
			OffsetPosition[0],OffsetPosition[1],OffsetPosition[2],
			relImuPosition.data());
	dRealVector3 absImuPosition; // The position of the IMU in the global coordinate system.
	for(unsigned int i=0; i<3; i++){
		absImuPosition[i]=bodyPosition.at(i)+relImuPosition.at(i);
	}
	return absImuPosition;
}

dRealMatrix3 Imu::GetRotation(){
	auto sharedBody=Body.lock();
	if(!sharedBody){ //TODO: This should throw an error!
		std::cout<<"The rotation of the IMU cannot be retrieved since the connected body does not exist anymore."<<std::endl;
	}
	dRealMatrix3 bodyRotation = sharedBody->GetRotation();
	dRealMatrix3 absRotation=boost::numeric::ublas::prod(bodyRotation,OffsetRotation);
	return absRotation;
}

dRealVector3 Imu::GetRelMagneticField(){
	boost::numeric::ublas::vector<dReal> relDirection(3);
	dRealVector3 temp;
	auto sharedBody=Body.lock();
	if(!sharedBody){ //TODO: This should throw an error!
		std::cout<<"The rotation of the IMU cannot be retrieved since the connected body does not exist anymore."<<std::endl;
	}
	dBodyVectorFromWorld (sharedBody->GetBodyId(), 1, 0, 0, temp.data());
	for(unsigned int i=0; i<3; i++){
		relDirection(i)=temp.at(i);
	};
	relDirection=boost::numeric::ublas::prod(InvOffsetRotation, relDirection);
	return boost::assign::list_of<dReal>(relDirection(0))(relDirection(1))(relDirection(2));
}

dRealVector3 Imu::GetRelAcceleration(){
	return RelAcceleration;
}

void Imu::PostSimulationStepUpdate(double deltaT){
	// Computation of the acceleration at the point of the sensor
	dRealVector3 acceleration;
	dRealVector3 detectedAcceleration; // The acceleration that is measured by the accelerometer
	dRealVector3 gravity={{0,0,-9.81}};
	
	auto sharedBody=Body.lock();
	if(!sharedBody){ //TODO: This should throw an error!
		std::cout<<"The rotation of the IMU cannot be retrieved since the connected body does not exist anymore."<<std::endl;
	}
	
	LastVelocity=Velocity;
	Bodies::RigidBody* rigidBody=dynamic_cast<Bodies::RigidBody*>(sharedBody.get());

	if(rigidBody){
		auto relMassOffset=rigidBody->GetRelMassOffset();
		dBodyGetPointVel(sharedBody->GetBodyId(), OffsetPosition.at(0)-relMassOffset.at(0), OffsetPosition.at(1)-relMassOffset.at(1), OffsetPosition.at(2)-relMassOffset.at(2), Velocity.data());
		dBodyVectorFromWorld(sharedBody->GetBodyId(), detectedAcceleration.at(0), detectedAcceleration.at(1), detectedAcceleration.at(2), RelAcceleration.data());
	}else{
		Velocity=boost::assign::list_of<double>(0)(0)(0);
		RelAcceleration=boost::assign::list_of<double>(0)(0)(0);
	};
	
	for(unsigned int i=0; i<3; i++){
		acceleration[i]=(Velocity.at(i)-LastVelocity.at(i))/deltaT;
		detectedAcceleration[i]=gravity[i]-acceleration[i];
	};

	boost::numeric::ublas::vector<dReal> tempRelAcceleration(3);
	for(unsigned int i=0; i<3; i++){
		tempRelAcceleration(i)=RelAcceleration[i];
	};
	tempRelAcceleration=boost::numeric::ublas::prod(InvOffsetRotation, tempRelAcceleration);
	for(unsigned int i=0; i<3; i++){
		RelAcceleration[i]=tempRelAcceleration(i);
	};
};

boost::shared_ptr<BfbMessage> Imu::ProcessMessage(boost::shared_ptr<const BfbMessage> Message) {
	auto reply=boost::shared_ptr<BfbMessage>(new BfbMessage(Message->GetRawData()));
	reply->SetDestination(Message->GetSource());
	reply->SetSource(Message->GetDestination());
	reply->SetBusAllocationFlag(false);
	reply->SetCommand(Message->GetCommand()+1);
	reply->SetPayload(boost::assign::list_of(0)(0));
	switch (Message->GetProtocol()){
		case BfbProtocolIds::IMU_SENSOR_PROT:
			switch (Message->GetCommand()){
				case 20: {// get relative acceleration
					auto tempRelAcceleration=RelAcceleration;
					for(unsigned int i=0; i<3; i++){
						tempRelAcceleration[i]*=1000;
					};
					reply->SetPayload(BfbFunctions::convertDoublesToBytes(boost::assign::list_of<double>(tempRelAcceleration.at(0))(tempRelAcceleration.at(1))(tempRelAcceleration.at(2)), 16, true));
					break;
				};
				case 40: {// get vector of magnetic field
					auto tempMagneticField=GetRelMagneticField();
					for(unsigned int i=0; i<3; i++){
						tempMagneticField[i]*=1000;
					};
					reply->SetPayload(BfbFunctions::convertDoublesToBytes(boost::assign::list_of<double>(tempMagneticField.at(0))(tempMagneticField.at(1))(tempMagneticField.at(2)), 16, true));
					break;
				};
				case 94: {// get global position
					auto tempPosition=GetPosition();
					for(unsigned int i=0; i<3; i++){
						tempPosition[i]*=1000000;
					};
					reply->SetPayload(BfbFunctions::convertDoublesToBytes(boost::assign::list_of<double>(tempPosition.at(0))(tempPosition.at(1))(tempPosition.at(2)), 32, true));
					break;
				};
				case 96: {// get global rotation
					auto tempRotation=GetRotation();
					dMatrix3 tempArray;
					HelperFunctions::DRealMatrix3ToDRealArray12(tempRotation, tempArray);
					dQuaternion tempQuaternion;
					dRtoQ (tempArray, tempQuaternion);
					double angle=2*acos(tempQuaternion[0]);
					std::vector<double> rotationAxis={{1,0,0}};
					if( angle != 0 ){
						rotationAxis[0]=tempQuaternion[1]/sqrt(1-pow(tempQuaternion[0],2));
						rotationAxis[1]=tempQuaternion[2]/sqrt(1-pow(tempQuaternion[0],2));
						rotationAxis[2]=tempQuaternion[3]/sqrt(1-pow(tempQuaternion[0],2));
					};
					double norm=sqrt(pow(rotationAxis[0],2)+pow(rotationAxis[1],2)+pow(rotationAxis[2],2));
					// Multiply the normalized rotation axis by the angle and scale it with the resolution of the sensor
					for(unsigned int i=0; i<3; i++){
						rotationAxis[i]=round(double(pow(2,24))*rotationAxis[i]/norm*angle);
					}
					auto payload=BfbFunctions::convertDoublesToBytes(rotationAxis, 32, true);
					reply->SetPayload(payload);
					break;
				};
			}
		break;
	}
	return reply;
}
