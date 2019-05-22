// STL includes
#include <stdexcept>
#include <string>

// Boost includes
#include <boost/assign/list_of.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>

// Own header files
#include "BfbClient.hpp"
#include "BioFlexRotatory.hpp"
#include "Bodies.hpp"
//#include "CollisionFeedback.hpp"
#include "PressureSensor.hpp"
#include "Universe.hpp"



/*! \brief Class constructor for the Universe class. */
unsigned int Universe::NumOfInstances=0;
Universe::Universe():
	FrictionCoefficientMap(StrFrictionCoefficientMap()),
	BodyMap(StrBodyMap()),
	BfbClientMap(UCharBfbClientMap()){	
	NumOfInstances++;
	if(NumOfInstances==1){ // If this is the first instance of the universe, the physics engine must be explicitly initialized.
		dInitODE();
	};
	
	CreateUniverse();
}



/*! \brief Class destructor */
Universe::~Universe()
{
	DestroyUniverse();
	
	NumOfInstances--;
	if(NumOfInstances==0){ // If the instance that is currently being destroyed was the last instance, the physics engine can be deinitialized.
		dCloseODE();
	};
}

void Universe::CreateUniverse(){
	dRandSetSeed(42); // In order to get a deteministic behaviour, the (pseudo-) random number generator is seeded with a constant number. "42" is chosen because it is the answer to the question "What is the perfect seed for the pseudo random number generator?".
	WorldId=dWorldCreate();	// create a new world in which dynamic bodies can be simulated. This will be the world all the objects in the universe will be placed into.
	SpaceId=dHashSpaceCreate(0);	// crate a new space for collision detection. The parameter defines the ID of a parent space the newly created space should be placed into. Setting this value to zero creates a new base space.
	ContactGroupId=dJointGroupCreate(0); 	// create the collision joint group. The parameter is deprecated, therefore should be set to zero.

	dWorldSetGravity(this->WorldId, this->Gravity.at(0), this->Gravity.at(1), this->Gravity.at(2));
	dWorldSetERP(this->WorldId, this->StdERP);
	dWorldSetCFM(this->WorldId, this->StdCFM);
	
	// Damping can be used in order to make the simulation more stable.
	//dWorldSetLinearDampingThreshold (WorldId, 0.0);
	//dWorldSetAngularDampingThreshold (WorldId, 0.0);
	//dWorldSetDamping (WorldId, 0.001,0.001);
	
	// The auto-disabling feature reduces the computational load for many objects that basically have a static equilibrium. These objects will then be considered to be static until they collide with a non-disabled object.
	//dWorldSetAutoDisableFlag(this->WorldId, false);
	//dWorldSetAutoDisableLinearThreshold( this->WorldId,0.01 );
	//dWorldSetAutoDisableAngularThreshold (this->WorldId, 0.01 );
	//dWorldSetAutoDisableSteps (this->WorldId, 10 );
}

void Universe::DestroyUniverse(){
	BodyMap.clear(); 		// Delete all rigid bodies within this universe.
	BfbClientMap.clear(); 	// Delete all BioFlex Rotatory drives within this universe.
	dWorldDestroy(this->WorldId); 	// Destroy the world that was used for the dynamics simulation.
	dSpaceDestroy(this->SpaceId); 	// Destroy the space that was used for collision detection.
}


void Universe::Clear(){
	DestroyUniverse();
	CreateUniverse();
}


// this is called by dSpaceCollide when two objects cin space are
// potentially colliding.
void Universe::NearCallback(void* InstancePointer, dGeomID primitive1, dGeomID primitive2)
{	
	if(!InstancePointer){
		throw std::runtime_error(std::string("Invalid InstancePointer."));
	}
	auto THIS=(Universe*)(InstancePointer);
	

	
	dContact contacts[THIS->MaxCollisionContacts]; 
	if ( dGeomIsSpace(primitive1) || dGeomIsSpace(primitive2) ) { 
		// colliding a space with something :
		dSpaceCollide2(primitive1, primitive2, InstancePointer, &NearCallback); 
		// collide all geoms internal to the space(s)
		if ( dGeomIsSpace(primitive1) )
			dSpaceCollide((dSpaceID)primitive1, InstancePointer, &NearCallback);
		if ( dGeomIsSpace (primitive2) )
			dSpaceCollide((dSpaceID)primitive2, InstancePointer, &NearCallback);
	} else {
		dBodyID bodyId1 = dGeomGetBody(primitive1);
		dBodyID bodyId2 = dGeomGetBody(primitive2);

		auto geomPrim1=(GeometricPrimitives::GeometricPrimitiveBase*)(dGeomGetData(primitive1)); 
		auto geomPrim2=(GeometricPrimitives::GeometricPrimitiveBase*)(dGeomGetData(primitive2)); 
		
		dReal friction=THIS->StdFrictionCoefficient;
		if(geomPrim1 && geomPrim2){
			friction=THIS->GetFrictionCoefficient(geomPrim1->GetMaterial(), geomPrim2->GetMaterial());
			// If it is necessary to also have a custom coefficient of restitution, it could be implemented just like the friction coefficient.
		};
		const unsigned int numOfContacts = dCollide (primitive1,primitive2,THIS->MaxCollisionContacts,&(contacts[0].geom),sizeof(dContact));
		for(unsigned int i=0;i<numOfContacts;i++){
			contacts[i].surface.mode = dContactApprox1;// | dContactBounce | dContactSoftCFM;
			// friction parameter
			contacts[i].surface.mu = friction;
			const dJointID contactJoint = dJointCreateContact(THIS->WorldId,THIS->ContactGroupId,&contacts[i]);
			dJointAttach (contactJoint,bodyId1,bodyId2);
			bool geom1UsesFeedback=geomPrim1 && geomPrim1->isCollisionFeedbackUsed();
			bool geom2UsesFeedback=geomPrim2 && geomPrim2->isCollisionFeedbackUsed();
			if(geom1UsesFeedback || geom2UsesFeedback){
				auto tempCollData=boost::make_shared<CollisionData>();
				THIS->CollisionDataList.push_back(tempCollData);
				tempCollData->JointId=contactJoint;
				tempCollData->ContactGeom=contacts[i].geom;
				if(geom1UsesFeedback){
					tempCollData->geom1=geomPrim1;
				};
				if(geom2UsesFeedback){
					tempCollData->geom2=geomPrim2;
				}
				dJointSetFeedback (contactJoint, &(tempCollData->JointFeedback));
			};
		};
	};
}

void Universe::Simulate(double deltaT)
{
	static double desiredTime=0;

	if(deltaT>0){
		desiredTime+=deltaT;
		unsigned int numOfIterations=floor((desiredTime-Time)*SimFrequency);
		for(unsigned int i=0;i<=numOfIterations;i++){
			// find collisions and add contact joints
			dSpaceCollide (SpaceId,this,&Universe::NearCallback);
			
			// simulate the dynamics
			dWorldQuickStep (this->WorldId ,1/(this->SimFrequency));  
			
			for(auto it=CollisionDataList.begin(); it!=CollisionDataList.end(); it++){
				CollisionFeedback tempFeedback1;
				CollisionFeedback tempFeedback2;
				dReal* tempPos=(*it)->ContactGeom.pos;
				dReal* tempNormal=(*it)->ContactGeom.normal;
				dReal* tempForce1=(*it)->JointFeedback.f1;
				dReal* tempForce2=(*it)->JointFeedback.f2;
				dReal* tempTorque1=(*it)->JointFeedback.t1;
				dReal* tempTorque2=(*it)->JointFeedback.t2;
				for(unsigned int i=0; i<3; i++){
					tempFeedback1.AbsPosition[i]=tempPos[i];
					tempFeedback2.AbsPosition[i]=tempPos[i];
					tempFeedback1.Normal[i]=tempNormal[i];
					tempFeedback2.Normal[i]=tempNormal[i];
					tempFeedback1.FeedbackForce[i]=tempForce1[i];
					tempFeedback2.FeedbackForce[i]=tempForce2[i];
					tempFeedback1.FeedbackTorque[i]=tempTorque1[i];
					tempFeedback2.FeedbackTorque[i]=tempTorque2[i];
				};
				if((*it)->geom1){
					(*it)->geom1->AddCollisionFeedback(tempFeedback1);
				};
				if((*it)->geom2){
					(*it)->geom2->AddCollisionFeedback(tempFeedback2);
				}
			};
			CollisionDataList.clear();
			
			// clear the collision joints for the next iteration.
			dJointGroupEmpty(this->ContactGroupId);
			
			// Update BioFlexRotatory Drives
			for(UCharBfbClientMap::iterator client=BfbClientMap.begin();client!=BfbClientMap.end();client++){
				client->second->PostSimulationStepUpdate(1/SimFrequency);
			};
			
			// update the local simulation time
			Time+=1.0/SimFrequency;
		};
	};
}

void Universe::SimulateUntill(double absTime){
	Simulate(absTime-Time);
}

/*
void Universe::Simulate(boost::posix_time::time_duration deltaT){
	Simulate(double(deltaT.total_microseconds())/pow(10,6));
}
*/

void Universe::Draw(){
	for(StrBodyMap::iterator i=BodyMap.begin();i!=BodyMap.end();i++){
		i->second->Draw();
	};
};

boost::shared_ptr<Bodies::RigidBody> Universe::AddRigidBody(	std::string name,
						dReal mass,
						dRealMatrix3 inertiaTensor,
						dRealVector3 initialPosition,
						dRealMatrix3 initialRotation,
						dRealVector3 relMassOffset,
						std::string material,
						std::bitset<32> categoryBits,
						std::bitset<32> collideBits){
	if(BodyMap.count(name)==1){
		throw std::invalid_argument("A Body with the name " + name + " already exists.");
	};
	boost::shared_ptr<Bodies::RigidBody> body(new Bodies::RigidBody(WorldId,
							SpaceId,
							name,
							mass,
							inertiaTensor,
							initialPosition,
							initialRotation,
							relMassOffset,
							material,
							categoryBits,
							collideBits));
	BodyMap[name]=static_cast<boost::shared_ptr<Bodies::BaseBody>>(body);
	return body;
};

boost::shared_ptr<Bodies::StaticBody> Universe::AddStaticBody(	std::string name,
							dRealVector3 initialPosition,
							dRealMatrix3 initialRotation,
							std::string material,
							std::bitset<32> categoryBits,
							std::bitset<32> collideBits){
	if(BodyMap.count(name)==1){
		throw std::invalid_argument("A Body with the name " + name + " already exists.");
	};
	boost::shared_ptr<Bodies::StaticBody> body(new Bodies::StaticBody(	SpaceId,
										name,
										initialPosition,
										initialRotation,
										material,
										categoryBits,
										collideBits));
	BodyMap[name]=body;
	return body;
};

boost::shared_ptr<Bodies::StaticBody> Universe::AddStaticBody(	std::string name,
								dReal mass,
								dRealMatrix3 inertiaTensor,
								dRealVector3 initialPosition,
								dRealMatrix3 initialRotation,
								dRealVector3 relMassOffset,
								std::string material,
								std::bitset<32> categoryBits,
								std::bitset<32> collideBits){
	(void)mass;
	(void)inertiaTensor;
	(void)relMassOffset;
	return AddStaticBody(	name,
				initialPosition,
				initialRotation,
				material,
				categoryBits,
				collideBits);
}

void Universe::DeleteBody(std::string name){
	auto it=BodyMap.find(name);
	if(it==BodyMap.end()){
		throw std::invalid_argument("A Body with the name " + name + " does not exist and can therefore not be deleted.");
	};
	BodyMap.erase(it);
}


boost::shared_ptr<BioFlexRotatory> Universe::AddBioFlexRotatory(	unsigned char bioFlexBusId,
																	dRealVector3 anchor,
																	dRealVector3 axis,
																	boost::shared_ptr<Bodies::BaseBody> bodyInput,
																	boost::shared_ptr<Bodies::BaseBody> bodyOutput,
																	dReal initialOutputAngle,
																	dReal springConstant,
																	dReal dampingConstant){
	if(BfbClientMap.count(bioFlexBusId)!=0){
		throw std::invalid_argument("A BioFlex Rotatory Drive with the bus ID " + boost::lexical_cast<std::string>(bioFlexBusId) + " does already exist.");
	};
	boost::shared_ptr<BioFlexRotatory> drive(new BioFlexRotatory(	WorldId,
									bioFlexBusId,
									anchor,
									axis,
									bodyInput,
									bodyOutput,
									initialOutputAngle,
									springConstant,
									dampingConstant));
	BfbClientMap[bioFlexBusId]=drive;
	return drive;
}; 

boost::shared_ptr<Imu> Universe::AttachImuToBody(unsigned char bioFlexBusId, 
						std::string nameOfBody,
						dRealVector3  offsetPosition, 
						dRealMatrix3  offsetRotation){
	if(BfbClientMap.count(bioFlexBusId)!=0){
		throw std::invalid_argument("A BioFlex Rotatory Drive with the bus ID " + boost::lexical_cast<std::string>(bioFlexBusId) + " does already exist.");
	};
	auto imu=boost::make_shared<Imu>(bioFlexBusId, 
					GetBody(nameOfBody),
					offsetPosition, 
					offsetRotation);
	BfbClientMap[bioFlexBusId]=imu;
	return imu;
}; 

boost::shared_ptr<PressureSensor> Universe::AttachPressureSensorToBody(unsigned char bioFlexBusId, 
						std::string nameOfBody,
						dRealVector3  offsetPosition, 
						dRealMatrix3  offsetRotation){
	if(BfbClientMap.count(bioFlexBusId)!=0){
		throw std::invalid_argument("A BioFlex Rotatory Drive with the bus ID " + boost::lexical_cast<std::string>(bioFlexBusId) + " does already exist.");
	};
	auto pressureSensor=boost::make_shared<PressureSensor>(bioFlexBusId, 
					GetBody(nameOfBody),
					offsetPosition=boost::assign::list_of(0)(0)(0), 
					offsetRotation=boost::numeric::ublas::identity_matrix<dReal>(3));
	BfbClientMap[bioFlexBusId]=pressureSensor;
	return pressureSensor;
}; 

boost::shared_ptr<Bodies::BaseBody> Universe::GetBody(std::string Name){
	return this->BodyMap.at(Name);
}

boost::shared_ptr<BfbClient> Universe::GetBfbClient(unsigned char BusId){
	return this->BfbClientMap.at(BusId);
};

dWorldID Universe::GetWorldId() { return this->WorldId; }

dSpaceID Universe::GetSpaceId() { return this->SpaceId; }

dReal Universe::GetTime() { return this->Time; }

dReal Universe::GetSimFrequency() { return this->SimFrequency; }

void Universe::SetSimFrequency(dReal newSimFrequency) { this->SimFrequency=newSimFrequency; }

dReal Universe::GetStdFrictionCoefficient() { return this->StdFrictionCoefficient; }

void Universe::SetStdFrictionCoefficient(dReal newStdFrictionCoefficient) { this->StdFrictionCoefficient=newStdFrictionCoefficient; }

dReal Universe::GetStdERP() { return this->StdERP; }

void Universe::SetStdERP( dReal newStdERP ){	this->StdERP = newStdERP; }

dReal Universe::GetStdCFM() { return this->StdCFM; }

void Universe::SetStdCFM( dReal newStdCFM ){	this->StdCFM = newStdCFM; }
 
std::string Universe::GetRigidBodyName(dBodyID Body)
{
	// iterate over each Object and compare its id with the one passed
	StrBodyMap::iterator i;
	for(i=this->BodyMap.begin(); i != this->BodyMap.end(); ++i)
	{
		if (i->second->GetBodyId() == Body) {
			return i->second->GetName();
		}
	}
	throw std::out_of_range("Could not find a body object with the assigned BodyID " + boost::lexical_cast<std::string>(Body));
}

std::string Universe::GetRigidBodyMaterial(dBodyID BodyId)
{
	// iterate over each Object and compare its id with the one passed
	StrBodyMap::iterator i;
	for(i=this->BodyMap.begin(); i != this->BodyMap.end(); ++i)
	{
		if (i->second->GetBodyId() == BodyId) {
			return i->second->GetMaterial();
		}
	}
	throw std::out_of_range("Could not find a body object with the assigned BodyID " + boost::lexical_cast<std::string>(BodyId));
}

void Universe::SetFrictionCoefficient(	std::string material1,
					std::string material2,
					dReal frictionCoefficient )
{
	// create string identifying a friction
	std::string materialCombination;
	if(material1>material2){
		materialCombination = material1 + "On" + material2;
	}else{
		materialCombination = material2 + "On" + material1;
	}
	this->FrictionCoefficientMap[materialCombination]=frictionCoefficient;
}

dReal Universe::GetFrictionCoefficient(std::string material1, 
				       std::string material2)
{
	// create string identifying a friction
	std::string materialCombination;
	if(material1>material2){
		materialCombination = material1 + "On" + material2;
	}else{
		materialCombination = material2 + "On" + material1;
	}
	dReal frictionCoefficient;
	try{
		frictionCoefficient=this->FrictionCoefficientMap.at(materialCombination);
	}catch(const std::out_of_range& oor) {
		frictionCoefficient=this->StdFrictionCoefficient;
	}
	return frictionCoefficient;
}
