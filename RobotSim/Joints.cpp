// STL includes
#include <iostream>
#include <string>

// Boost includes
#include <boost/math/constants/constants.hpp>
#include <boost/weak_ptr.hpp>

// Own header files
#include "Bodies.hpp"
#include "HelperFunctions.hpp"
#include "Joints.hpp"
#include "OdeDrawstuff.hpp"

static dReal PI=boost::math::constants::pi<dReal>();

template <typename T>
T clip(const T& n, const T& lower, const T& upper) {
  return std::max(lower, std::min(n, upper));
}


namespace Joints{
	BaseJoint::BaseJoint( std::string name, 
			      boost::weak_ptr<Bodies::BaseBody> body1,
			      boost::weak_ptr<Bodies::BaseBody> body2):
				Name(name), 
				Body1(body1), 
				Body2(body2){
	};
	BaseJoint::~BaseJoint(){
		dJointDestroy(this->JointId);
	};
	void BaseJoint::Attach(boost::weak_ptr<Bodies::BaseBody> body1, boost::weak_ptr<Bodies::BaseBody> body2){
		this->Body1=body1;
		this->Body2=body2;
		auto sharedBody1=body1.lock();
		auto sharedBody2=body2.lock();
		if(!sharedBody1 || !sharedBody2 || sharedBody1==sharedBody2){std::cout<<"this should throw an error"<<std::endl;};// TODO: This should throw an error!
		if(sharedBody1!=nullptr && sharedBody2!=nullptr){
			dJointAttach(this->JointId, sharedBody1->GetBodyId(), sharedBody2->GetBodyId());
		}else if(sharedBody1!=nullptr){
			dJointAttach(this->JointId, sharedBody1->GetBodyId(), 0);
			//std::cout<<"Body1 is Fixed."<<std::endl;
		}else if(sharedBody2!=nullptr){
			dJointAttach(this->JointId, 0, sharedBody2->GetBodyId());
			//std::cout<<"Body2 is Fixed."<<std::endl;
		};
	};
	
	boost::weak_ptr<Bodies::BaseBody > BaseJoint::GetBody1(){return this->Body1;};
	boost::weak_ptr<Bodies::BaseBody> BaseJoint::GetBody2(){return this->Body2;};
	
	bool BaseJoint::IsAttached(){
		if(Body1.lock()->GetBodyId()!=Body2.lock()->GetBodyId()){
			return true;
		}else{
			return false;
		};
	};
	
	std::string BaseJoint::GetName(){return this->Name;};
	
	dJointID BaseJoint::GetJointId(){return this->JointId;}; 
	
	Hinge::Hinge(	dWorldID worldId,
			std::string name,
			dRealVector3 anchor,
			dRealVector3 axis,
			boost::weak_ptr<Bodies::BaseBody> body1,
			boost::weak_ptr<Bodies::BaseBody> body2,
			dReal angularOffset=0): 
				BaseJoint(name, body1, body2),
				AngularOffset(angularOffset){
		this->JointId=dJointCreateHinge (worldId, 0);
		
		Attach(body2, body1);
		SetAnchor(anchor);
		SetAxis(axis);
	};
	

	void Hinge::SetAnchor(dRealVector3 anchor){
		dJointSetHingeAnchor(this->JointId, anchor.at(0), anchor.at(1), anchor.at(2));
		//std::cout<<"Anchor: "<< Anchor[0]<<","<< Anchor[1]<<"," << Anchor[2]<<"," << std::endl;
	};
	
	dRealVector3 Hinge::GetAnchor( ){
		dVector3 temp;
		dJointGetHingeAnchor(this->JointId, temp);
		return HelperFunctions::DRealArrayToDRealVector3(temp);
	};
	
	void Hinge::SetAxis(dRealVector3 axis){
		dJointSetHingeAxis(this->JointId, axis.at(0), axis.at(1), axis.at(2));
	};

	dRealVector3 Hinge::GetAxis( ){
		dRealVector3 temp;
		dJointGetHingeAxis(this->JointId, temp.data());
		return temp;
	};
	
	dReal Hinge::GetAngle(){
		dReal tempAngle=dJointGetHingeAngle(JointId) + AngularOffset;
		if(tempAngle>PI){
			tempAngle-=2*PI;
		}else if(tempAngle<-PI){
			tempAngle+=2*PI;
		};
		return tempAngle;
	};



}//end namespace
