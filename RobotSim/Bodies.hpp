#ifndef BODY_HPP
#define BODY_HPP

// STL includes
#include <bitset>
#include <iostream>
#include <functional>
#include <list>
#include <string>

// Boost includes
#include <boost/assign/list_of.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>
#include <boost/utility.hpp>

// Own header files
//#include "CollisionFeedback.hpp"
#include "DataTypes.hpp"
#include "GeometricPrimitives.hpp"
#include "OdeDrawstuff.hpp"
namespace Bodies{

/*! \brief Objects in the simulated world are represented by RigidBodies (for movable objects) and StaticBodies (for non-movable objects).
 *
 *	Each object in an ODE simulation has a body for the dynamics simulation. 
 * 	These bodies represent points of mass (with a certain inertia). However,
 * 	they do not possess a spatial extent. In order to give them a spatial 
 * 	representation, e.g. for collision detection, ODE uses geometric primitives.
 * 	These primitives can be associated with a body, thus sharing a certain position
 * 	and rotation. 
 * 	In this simulation, the bodies are divided into movable and non-movable bodies.
 * 	Former are called RigidBodies since they are not deformable, latter are called
 * 	Static Bodies since they are neither deformable nor movable. 
 * 	Using appropriate methods, geometric primitives (spheres, cylinders, capsules etc.) 
 * 	can be associated with these bodies.
 * 	In addition to the primitives that can be associated with the movable bodies,
 * 	the static bodies can be associated also with planes and heightfields.
 *	
 */
 
typedef std::list< boost::shared_ptr<GeometricPrimitives::GeometricPrimitiveBase> > GeometricPrimitivesList;
 
class BaseBody{
	protected:
		dBodyID			BodyId=0;	
		dSpaceID 		SpaceId;
		std::string 		Name;	
		
		dRealVector3 		InitialPosition; 
		dRealMatrix3 		InitialRotation;	
		
		std::string 		Material;
		
		std::bitset<32> 	CategoryBits;
		std::bitset<32> 	CollideBits;
		
		//! A list of primitives composing this object!
		GeometricPrimitivesList GeomPrimList;
		
		BaseBody(	dSpaceID 	spaceId,
				std::string	name,
				dRealVector3 	initialPosition,
				dRealMatrix3 	initialRotation,
				std::string 	material,
	    			std::bitset<32> categoryBits,
				std::bitset<32> collideBits);
		virtual ~BaseBody();
		
		virtual void SetPosAndRotOfPrimitive(boost::shared_ptr<GeometricPrimitives::GeometricPrimitiveBase> primitive, dRealVector3 offsetPosition, dRealMatrix3 offsetRotation);
		std::vector<std::function<void (const CollisionFeedback collisionFeedback)>> CollisionFeedbackForwardFunctions={};
	public:
		BaseBody(const BaseBody&) = delete;
		BaseBody & operator=(const BaseBody&) = delete;
		
		/*!	\brief Get the space the primitives of this object belong to.
		*
		*	All primitives which account for a collision are added to a subspace to
		*	speed-up collision handling.
		*	\return the ID of the sub space for the primitives associated with this
		*		object.
		*/
		dSpaceID GetSpaceId( );

		/*! \brief Get the ID of this RigidBody.
		*
		*	\return the ID of this GoemetricBody (type dBodyID).
		*/
		dBodyID GetBodyId( void );
		void AdministerNewGeometricPrimitive(boost::shared_ptr<GeometricPrimitives::GeometricPrimitiveBase> primitive);
		void AddSphere( dReal radius=0.5, 
				dRealVector3 offsetPosition=boost::assign::list_of<dReal>(0)(0)(0), 
				dRealMatrix3 offsetRotation=boost::numeric::ublas::identity_matrix<dReal>(3), 
				doubleArray3 color=boost::assign::list_of<dReal>(1)(0)(0));
		void AddBox( 	dRealVector3 dimension=boost::assign::list_of<dReal>(1)(1)(1), 
				dRealVector3 offsetPosition=boost::assign::list_of<dReal>(0)(0)(0), 
				dRealMatrix3 offsetRotation=boost::numeric::ublas::identity_matrix<dReal>(3), 
				doubleArray3 color=boost::assign::list_of<dReal>(1)(0)(0) );
		void AddCapsule(dReal radius=0.5, 
				dReal length=1, 
				dRealVector3 offsetPosition=boost::assign::list_of<dReal>(0)(0)(0), 
				dRealMatrix3 offsetRotation=boost::numeric::ublas::identity_matrix<dReal>(3), 
				doubleArray3 color=boost::assign::list_of<dReal>(1)(0)(0) );
		void AddCylinder(dReal radius=0.5, 
				 dReal length=1, 
				dRealVector3 offsetPosition=boost::assign::list_of<dReal>(0)(0)(0), 
				dRealMatrix3 offsetRotation=boost::numeric::ublas::identity_matrix<dReal>(3), 
				doubleArray3 color=boost::assign::list_of<dReal>(1)(0)(0) );
		void AddTriMesh(std::vector<dRealVector3> vertices, 
				std::vector<std::array<unsigned int,3>> faceIndices, 
				dRealVector3 offsetPosition=boost::assign::list_of<dReal>(0)(0)(0), 
				dRealMatrix3 offsetRotation=boost::numeric::ublas::identity_matrix<dReal>(3), 
				doubleArray3 color=boost::assign::list_of<dReal>(1)(0)(0));
		
		void AddHeightfield(	dReal xLength, 
					dReal yLength, 
					boost::numeric::ublas::matrix<dReal> zData, 
					bool tile=false, 
					dRealVector3 offsetPosition=boost::assign::list_of<dReal>(0)(0)(0), 
					dRealMatrix3 offsetRotation=boost::numeric::ublas::identity_matrix<dReal>(3), 
					doubleArray3 color=boost::assign::list_of<dReal>(1)(0)(0));
		
		
		
		void AddCollisionFeedback(const CollisionFeedback collisionFeedback);
		
		void RouteCollisionFeedbacksTo(std::function<void (const CollisionFeedback collisionFeedback)> callbackFunction);
		
		void Draw();
		
		/*! \brief	Rotate the RigidBody object
		*
		*	To rotate an object around a center point several parameters are
		*	needed:\n
		*	- The angle of rotation\n
		*	- The axis of rotation\n
		*	- The center point of the rotation\n\n
		*	With these parameters a rotation matrix can be constructed which
		*	if applied to the object rotates it by the angle around the center
		*	point and the axis of rotation to its new position.
		*
		*	\param angle The angle of rotation.
		*	\param axis The direction of the axis to rotate around.
		*	\param anchor The position of the center point of
		*		rotation.
		*/
		/*
		void RotateAroundAxis(dReal angle, dRealVector3 axis, dRealVector3 anchor);
		*/

		/*! \brief Get the name of this RigidBody.
		*
		*	\return the name of this Body.
		*/
		std::string GetName( );

		/*! \brief Get the material of this RigidBody.
		*
		*	\return the material of this Body.
		*/
		std::string GetMaterial( );
		
		/*! \brief Get the position of this RigidBody.
		*
		*	ODE does not support different positions for the center of mass and the
		*	origin of the object. To overcome this limitation a workaround was
		*	added, everytime the position of an object is needed. Therefore the mass
		*	offset is saved in a class variable and is used everytime the object's
		*	position is needed.
		*
		*/
		virtual dRealVector3 GetPosition( )=0;
		
		/*!	\brief Get the rotation matrix of this object
		*
		*/
		virtual dRealMatrix3 GetRotation( )=0;
};



class RigidBody: public BaseBody
{
	private:
		
		dWorldID		WorldId;
		
		dReal Mass;	//!< The mass of this RigidBody
		
		dRealVector3 		RelMassOffset=boost::assign::list_of<dReal>(0)(0)(0);
		
		dRealMatrix3 InertiaTensor;
		/* \brief Offset of the center of mass to the origin of 
		*		the RigidBody.
		*
		*	In ODE the center of mass and the body's origin have to be at the same location. To bypass this restriction each RigidBody object features a variable \c InitialMassOffset of type dRealVector3. This vector describes the distance of the center of mass to the origin of the body. Although the origin never changes its position	the InitialMassOffset allows the positioning of primitives of which the body is constructed in relation to the virtual origin with a location different than the location of the center of mass.
		*/
		//dRealVector3 RelPositionOffset;

		//void SetPosAndRotOfPrimitive(boost::shared_ptr<GeometricPrimitives::GeometricPrimitiveBase> primitive, dRealVector3 offsetPosition, dRealMatrix3 offsetRotation);
	public:
		RigidBody(const RigidBody&) = delete;
		RigidBody & operator=(const RigidBody&) = delete;
		
		RigidBody(	dWorldID worldId,
				dSpaceID spaceId,
				std::string name,
				dReal mass=1,
				dRealMatrix3 inertiaTensor=boost::numeric::ublas::identity_matrix<dReal>(3),
				dRealVector3 initialPosition=boost::assign::list_of<dReal>(0)(0)(0),
				dRealMatrix3 initialRotation=boost::numeric::ublas::identity_matrix<dReal>(3),
				dRealVector3 relMassOffset=boost::assign::list_of<dReal>(0)(0)(0),
				std::string material="default",
				std::bitset<32> categoryBits=0xFFFF,
				std::bitset<32> collideBits=0xFFFF);
		
		virtual ~RigidBody();
		
		virtual dRealVector3 GetPosition( );
		
		virtual void SetPosAndRotOfPrimitive(boost::shared_ptr<GeometricPrimitives::GeometricPrimitiveBase> primitive, dRealVector3 offsetPosition, dRealMatrix3 offsetRotation);
		
		/*! \brief Set the position of this RigidBody.
		*
		*	ODE does not support different positions for the center of mass and the
		*	origin of the object yet. To overcome this limitation a workaround was
		*	added, everytime the position of an object is needed. Therefore the mass
		*	offset is saved in a class variable and is used everytime the object's
		*	position is needed.
		*
		*	\param Position a a pointer to a vector (array) with the new position
		*		vector.
		*/
		virtual void SetPosition(dRealVector3 NewPosition);

		virtual dRealMatrix3 GetRotation( );
		
		/*!	\brief Set the orientation of this object (rotation matrix)
		*
		*	\param RotationMatrix A 4x3 rotation matrix (no translation) used
		*		to set the orientation of the object
		*/
		virtual void SetRotation(dRealMatrix3 NewRotationMatrix);
		
		dRealVector3 GetRelMassOffset();
};

class StaticBody: public BaseBody{
	private: 

	public:
		StaticBody(const StaticBody&) = delete;
		StaticBody & operator=(const StaticBody&) = delete;
		
		StaticBody(	dWorldID worldId,
				dSpaceID spaceId,
				std::string name,
				dReal mass,
				dRealMatrix3 inertiaTensor=boost::numeric::ublas::identity_matrix<dReal>(3),
				dRealVector3 initialPosition=boost::assign::list_of<dReal>(0)(0)(0),
				dRealMatrix3 initialRotation=boost::numeric::ublas::identity_matrix<dReal>(3),
				dRealVector3 relMassOffset=boost::assign::list_of<dReal>(0)(0)(0),
				std::string material="default",
				std::bitset<32> categoryBits=0xFFFF,
				std::bitset<32> collideBits=0xFFFF);
		
		StaticBody(	dSpaceID spaceId,
				std::string name,
				dRealVector3 initialPosition=boost::assign::list_of<dReal>(0)(0)(0),
				dRealMatrix3 initialRotation=boost::numeric::ublas::identity_matrix<dReal>(3),
				std::string material="default",
				std::bitset<32> categoryBits=0xFFFF,
				std::bitset<32> collideBits=0xFFFF);
		
		virtual dRealVector3 GetPosition( );
		virtual dRealMatrix3 GetRotation( );
		
		void AddPlane(	dRealVector3 offsetPosition=boost::assign::list_of<dReal>(0)(0)(0), 
				dRealMatrix3 offsetRotation=boost::numeric::ublas::identity_matrix<dReal>(3), 
				doubleArray3 color=boost::assign::list_of<dReal>(1)(0)(0));
		

};

};
#endif
