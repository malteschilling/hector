// STL includes
#include <string.h>

// Boost includes
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <boost/math/constants/constants.hpp>

// Own header files
#include "Bodies.hpp"
//#include "CollisionFeedback.hpp"
#include "DataTypes.hpp"
#include "HelperFunctions.hpp"
#include "OdeDrawstuff.hpp"

static unsigned int 	NumOfInstance=0;	// This variable is used to track the number of instances THAT HAVE BEEN CREATED THROUGHOUT THE SIMULATION. 
namespace Bodies{
BaseBody::BaseBody(	dSpaceID 	spaceId,
			std::string	name,
			dRealVector3 	initialPosition,
			dRealMatrix3 	initialRotation,
			std::string 	material,
			std::bitset<32> categoryBits,
			std::bitset<32> collideBits):
				SpaceId(spaceId),
				Name(name),
				InitialPosition(initialPosition),
				InitialRotation(initialRotation),
				Material(material),
				CategoryBits(categoryBits),
				CollideBits(collideBits),
				GeomPrimList(GeometricPrimitivesList(0)){
	NumOfInstance++;
	if(Name==""){
		Name=std::string("BodyNr").append(boost::lexical_cast<std::string>(NumOfInstance));
	}
	
	/*	Create a sub-space to hold the primitves for this RigidBody and do
		the collision checks. A quadspace has proven to be very bad here (according to Malte Marwedel)*/
	SpaceId = dHashSpaceCreate(spaceId);

	/*	Set category and collide bits. Both 32 bit wide. This feature is 
		optional but can assist collision detection. Each geometry can be a
		member of a category (category bit). The collide bit defines which
		categories the geometry can collide with. */
	//dGeomSetCollideBits((dGeomID)SpaceId, CollideBits.to_ulong());
	//dGeomSetCategoryBits((dGeomID)SpaceId, CategoryBits.to_ulong());

	// create a body in the world and save pointer to that body
	
	

	// rotate the body to its initial orientation
	//SetRotation(InitialRotation); 

	// set the body's position to its initial position
	//SetPosition(InitialPosition);
};

/*!	\brief	Class destructor */
BaseBody::~BaseBody(){
	GeomPrimList.clear();
	dSpaceDestroy(SpaceId);
}

void BaseBody::RouteCollisionFeedbacksTo(std::function<void (const CollisionFeedback collisionFeedback)> callbackFunction){
	if(CollisionFeedbackForwardFunctions.size()==0){
		for(auto g=GeomPrimList.begin(); g!=GeomPrimList.end(); g++){
			(*g)->RouteCollisionFeedbacksTo(boost::bind(&BaseBody::AddCollisionFeedback, this, _1));
		};
	};
	CollisionFeedbackForwardFunctions.push_back(callbackFunction);
}

void BaseBody::AddCollisionFeedback(const CollisionFeedback collisionFeedback){
	for(auto it=CollisionFeedbackForwardFunctions.begin(); it!=CollisionFeedbackForwardFunctions.end(); it++){
		(*it)(collisionFeedback);
	};
}

void BaseBody::AdministerNewGeometricPrimitive(boost::shared_ptr<GeometricPrimitives::GeometricPrimitiveBase> primitive){
	this->GeomPrimList.push_back(primitive);
	if(CollisionFeedbackForwardFunctions.size()>0){
		primitive->RouteCollisionFeedbacksTo(boost::bind(&BaseBody::AddCollisionFeedback, this, _1));
	}
};

void BaseBody::AddSphere( dReal Radius, dRealVector3 offsetPosition, dRealMatrix3 offsetRotation, doubleArray3 color ){
	boost::shared_ptr<GeometricPrimitives::Sphere> primitive(new GeometricPrimitives::Sphere(Radius, color, SpaceId));
	AdministerNewGeometricPrimitive(primitive);
	SetPosAndRotOfPrimitive(primitive, offsetPosition, offsetRotation);
};

void BaseBody::AddBox( dRealVector3 Dimension, dRealVector3 offsetPosition, dRealMatrix3 offsetRotation, doubleArray3 color ){
	boost::shared_ptr<GeometricPrimitives::Box> primitive(new GeometricPrimitives::Box(Dimension, color, SpaceId));
	AdministerNewGeometricPrimitive(primitive);
	SetPosAndRotOfPrimitive(primitive, offsetPosition, offsetRotation);
};

void BaseBody::AddCapsule(dReal radius, dReal length, dRealVector3 offsetPosition, dRealMatrix3 offsetRotation, doubleArray3 color ){
	boost::shared_ptr<GeometricPrimitives::Capsule> primitive(new GeometricPrimitives::Capsule( radius, length, color, SpaceId));
	AdministerNewGeometricPrimitive(primitive);
	SetPosAndRotOfPrimitive(primitive, offsetPosition, offsetRotation);
};

void BaseBody::AddCylinder(dReal radius, dReal length, dRealVector3 offsetPosition, dRealMatrix3 offsetRotation, doubleArray3 color ){
	boost::shared_ptr<GeometricPrimitives::Cylinder> primitive(new GeometricPrimitives::Cylinder( radius, length, color, SpaceId));
	AdministerNewGeometricPrimitive(primitive);
	SetPosAndRotOfPrimitive(primitive, offsetPosition, offsetRotation);
};

void BaseBody::AddTriMesh(std::vector<dRealVector3> vertices, std::vector<std::array<unsigned int,3>> faceIndices, dRealVector3 offsetPosition, dRealMatrix3 offsetRotation, doubleArray3 color){
	boost::shared_ptr<GeometricPrimitives::TriMesh> primitive(new GeometricPrimitives::TriMesh(vertices, faceIndices, color, SpaceId));
	AdministerNewGeometricPrimitive(primitive);
	SetPosAndRotOfPrimitive(primitive, offsetPosition, offsetRotation);
};

void BaseBody::AddHeightfield(dReal xLength, dReal yLength, boost::numeric::ublas::matrix<dReal> zData, bool tile, dRealVector3 offsetPosition, dRealMatrix3 offsetRotation, doubleArray3 color){
	auto primitive=boost::make_shared<GeometricPrimitives::Heightfield>(xLength, yLength, zData, tile, color, SpaceId);
	AdministerNewGeometricPrimitive(primitive);
	auto tempRot=HelperFunctions::RotateAroundAxis(boost::math::constants::pi< double >()/2, boost::assign::list_of<dReal>(1)(0)(0));
	offsetRotation=boost::numeric::ublas::prod(offsetRotation,tempRot);
	SetPosAndRotOfPrimitive(primitive, offsetPosition, offsetRotation);
};

void BaseBody::Draw(){
	for(GeometricPrimitivesList::iterator i=this->GeomPrimList.begin(); i!=this->GeomPrimList.end(); i++){
		(*i)->Draw();
	};
};
/*
void BaseBody::RotateAroundAxis(	dReal angle,
					dRealVector3 axis,
					dRealVector3 anchor)
{	
	dRealMatrix3 rotationMatrix=HelperFunctions::RotateAroundAxis(angle, axis);
	dRealMatrix3 currentRotation=this->GetRotation();
	dRealVector3 position=this->GetPosition();
	dRealMatrix3 newRotationMatrix=boost::numeric::ublas::prod(rotationMatrix,currentRotation);
	SetRotation(newRotationMatrix);
	//	object is now rotated but needs to be translated
	boost::numeric::ublas::bounded_vector<dReal,3> posRelToAnchor;
	for(unsigned int i=0;i<3;i++){posRelToAnchor[i]=position[i]-anchor[i];};
	posRelToAnchor=boost::numeric::ublas::prod(rotationMatrix,posRelToAnchor);
	for(unsigned int i=0;i<3;i++){position[i]=posRelToAnchor(i)+anchor[i];};
	SetPosition(position);
}
*/
std::string BaseBody::GetName() { return Name; }

std::string BaseBody::GetMaterial() { return Material; }

dBodyID BaseBody::GetBodyId() { return BodyId; }

dSpaceID BaseBody::GetSpaceId() { return SpaceId; }

void BaseBody::SetPosAndRotOfPrimitive(boost::shared_ptr<GeometricPrimitives::GeometricPrimitiveBase> primitive, dRealVector3 offsetPosition, dRealMatrix3 offsetRotation){
	dGeomID tempGeomId=primitive->GetGeomId();
	boost::numeric::ublas::vector<dReal> tempVec(3);
	for(unsigned int i=0; i<3; i++){
		tempVec(i)=offsetPosition[i];
	};
	tempVec=boost::numeric::ublas::prod(InitialRotation, tempVec);
	dGeomSetPosition(tempGeomId,tempVec(0)+InitialPosition[0],tempVec(1)+InitialPosition[1],tempVec(2)+InitialPosition[2]);
	dRealMatrix3 absRotation=boost::numeric::ublas::prod(InitialRotation, offsetRotation);
	dMatrix3 offsetRotationArray;
	HelperFunctions::DRealMatrix3ToDRealArray12( absRotation, offsetRotationArray);
	dGeomSetRotation (tempGeomId, offsetRotationArray);
};



RigidBody::RigidBody(	dWorldID worldId,
			dSpaceID spaceId,
			std::string name,
			dReal mass,
			dRealMatrix3 inertiaTensor,
			dRealVector3 initialPosition,
			dRealMatrix3 initialRotation,
			dRealVector3 relMassOffset,
			std::string material,
			std::bitset<32> categoryBits,
			std::bitset<32> collideBits):
			BaseBody(	spaceId,
					name,
					initialPosition,
					initialRotation,
					material,
					categoryBits,
					collideBits),
			WorldId(worldId),
			Mass(mass),
			RelMassOffset(relMassOffset),
			InertiaTensor(inertiaTensor){
		BodyId=dBodyCreate(worldId),
		
		// set autodisable. Object will be disabled if idle for long enough
		dBodySetAutoDisableFlag(BodyId, true); //default would be on
		auto tempMass=HelperFunctions::CreateDMass(Mass, InertiaTensor);
		// set mass for body if this body has a mass.
		dBodySetMass(BodyId,&tempMass);
		
		dBodySetData(BodyId,this);
		
		// rotate the body to its initial orientation
		SetRotation(initialRotation); 
		// set the body's position to its initial position
		SetPosition(initialPosition);
}

/*!	\brief	Class destructor */
RigidBody::~RigidBody(){
	dBodyDestroy(this->BodyId);
}



dRealVector3 RigidBody::GetPosition( void ){
	dRealVector3 Position;
	dBodyGetRelPointPos (BodyId, -RelMassOffset[0], -RelMassOffset[1], -RelMassOffset[2], Position.data());
	return Position;
}

void RigidBody::SetPosition( dRealVector3 newPosition){
	dVector3 AbsMassOffset; // The body's mass offset in world coordinates
	dBodyVectorToWorld (BodyId,  RelMassOffset[0], RelMassOffset[1], RelMassOffset[2], AbsMassOffset);
	dBodySetPosition (BodyId, newPosition[0]+AbsMassOffset[0] , newPosition[1]+AbsMassOffset[1] , newPosition[2]+AbsMassOffset[2] );
}

dRealMatrix3 RigidBody::GetRotation( ){
	const dReal* tempRotation=dBodyGetRotation(BodyId);
	dMatrix3 tempMatrix;
	for(unsigned int i=0;i<12;i++){tempMatrix[i]=tempRotation[i];};
	return HelperFunctions::DRealArray12ToDRealMatrix3(tempMatrix);
}

void RigidBody::SetRotation( dRealMatrix3 newRotationMatrix ){
	dMatrix3 rotationArray;
	HelperFunctions::DRealMatrix3ToDRealArray12( newRotationMatrix, rotationArray);
	dRealVector3 Position=this->GetPosition();
	dBodySetRotation(BodyId, rotationArray);
	SetPosition(Position);
}

void RigidBody::SetPosAndRotOfPrimitive(boost::shared_ptr<GeometricPrimitives::GeometricPrimitiveBase> primitive, dRealVector3 offsetPosition, dRealMatrix3 offsetRotation){
	dGeomID tempGeomId=primitive->GetGeomId();
	dGeomSetBody(tempGeomId,BodyId);
	dGeomSetOffsetPosition(tempGeomId,offsetPosition[0]-RelMassOffset[0],offsetPosition[1]-RelMassOffset[1],offsetPosition[2]-RelMassOffset[2]);
	dMatrix3 offsetRotationArray;
	HelperFunctions::DRealMatrix3ToDRealArray12( offsetRotation, offsetRotationArray);
	dGeomSetOffsetRotation (tempGeomId, offsetRotationArray);
};

dRealVector3 RigidBody::GetRelMassOffset(){
	return RelMassOffset;
}




StaticBody::StaticBody(	dWorldID worldId,
			dSpaceID spaceId,
			std::string name,
			dReal mass,
			dRealMatrix3 inertiaTensor,
			dRealVector3 initialPosition,
			dRealMatrix3 initialRotation,
			dRealVector3 relMassOffset,
			std::string material,
			std::bitset<32> categoryBits,
			std::bitset<32> collideBits):
				StaticBody(	spaceId,
						name,
						initialPosition,
						initialRotation,
						material,
						categoryBits,
						collideBits){
};

StaticBody::StaticBody(	dSpaceID spaceId,
			std::string name,
			dRealVector3 initialPosition,
			dRealMatrix3 initialRotation,
			std::string material,
			std::bitset<32> categoryBits,
			std::bitset<32> collideBits):
				BaseBody(	spaceId,
						name,
						initialPosition,
						initialRotation,
						material,
						categoryBits,
						collideBits){
};

dRealVector3 StaticBody::GetPosition( ){
	return InitialPosition;
}

dRealMatrix3 StaticBody::GetRotation( ){
	return InitialRotation;
}

void StaticBody::AddPlane(dRealVector3 offsetPosition, dRealMatrix3 offsetRotation, doubleArray3 color){
	boost::numeric::ublas::vector<dReal> tempOffsetPosition(3);
	for(unsigned int i=0; i<3; i++){
		tempOffsetPosition(i)=offsetPosition[i];
	};
	tempOffsetPosition=boost::numeric::ublas::prod(InitialRotation, tempOffsetPosition);
	for(unsigned int i=0; i<3; i++){
		tempOffsetPosition(i)=tempOffsetPosition[i]+InitialPosition[i];
	};
	
	dRealMatrix3 absRotation=boost::numeric::ublas::prod(InitialRotation, offsetRotation);
	boost::numeric::ublas::vector<dReal> normal=boost::numeric::ublas::unit_vector<dReal>(3,2);
	normal=boost::numeric::ublas::prod(absRotation, normal);
	dReal distance=boost::numeric::ublas::inner_prod(tempOffsetPosition, normal);
	auto primitive=boost::make_shared<GeometricPrimitives::Plane>(boost::assign::list_of<dReal>(normal(0))(normal(1))(normal(2)), distance, color, SpaceId);
	AdministerNewGeometricPrimitive(primitive);
}

};