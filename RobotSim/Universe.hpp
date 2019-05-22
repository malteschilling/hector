#ifndef UNIVERSE_HPP
#define UNIVERSE_HPP

// STL includes
#include <map>
#include <string>

// Boost includes
#include <boost/assign/list_of.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/variant.hpp>

// Own header files
#include "BfbClient.hpp"
#include "BioFlexRotatory.hpp"
#include "Bodies.hpp"
#include "Imu.hpp"
#include "PressureSensor.hpp"
#include "OdeDrawstuff.hpp"


/** \brief A special map that uses strings as keys and shared pointers to rigid bodies as elements. */
typedef std::map<std::string, boost::shared_ptr<Bodies::BaseBody> > StrBodyMap;
/** \brief A special map that uses unsigned characters as keys and shared pointers to BioFlex Rotatory drives as elements. */
typedef std::map<unsigned char, boost::shared_ptr<BfbClient> > UCharBfbClientMap;
/** \brief A special map that uses strings as keys and friction coefficients (real numbers) as elements. */
typedef std::map<std::string, dReal > StrFrictionCoefficientMap;

struct CollisionData{
	CollisionData():
		JointFeedback(),
		ContactGeom(){
	};
	dJointID JointId=0;
	dJointFeedback JointFeedback;
	dContactGeom ContactGeom;
	GeometricPrimitives::GeometricPrimitiveBase* geom1=nullptr;
	GeometricPrimitives::GeometricPrimitiveBase* geom2=nullptr;
};



/*! \brief The simulation environment is represented by the Universe class.
 *
 *	Each object in an ODE simulation is located inside a universe. There should
 *	be only one universe per simulation at the same time. The Universe object
 *	holds all other object and special parameters which define for example the
 *	gravity within the simulation or the time between two discrete simulation
 *	steps. For more information about the parameters defined by the universe see the
 *	variable descriptions of the class.
 */
class Universe {
private:
	static unsigned int 	NumOfInstances;			//!< This variable is shared by a instances of the universe class. It counts the number of instances. This is used to initialize the physics engine for the first instance and to deinitialize it when the last instance is being deleted.
	dRealVector3 		Gravity		=boost::assign::list_of<dReal>(0)(0)(-9.81);	//!< The gravity within the universe. Since it's a vector, it cannot be initalized in the header file.
	dReal 			SimFrequency	=1000;		//!< The simulation frequency [Hz]. 1/SimFrequency defines the timestep between two iterations of the physics engine. It refers to simulation time, not to real time. 
		
	/** \brief Error Reduction Parameter
	 * This parameter controls the way the simulation deals with joints whose constraints are not met exactly (e.g. a ball and socket joint in which the ball and the socket habe drifted apart).
	 * Dependend on the distance the two bodies have drifted apart, a restoring force is applied between the bodies. A value of 0 sets this force to zero. Setting the parameter to 1 lets the simulation 
	 * try to completely correct the error during the next simulation step. This, however, might very well lead to oscillations. A value between 0.2 and 0.8 is recommended according to the ode manual.
	 * The default value is 0.2. 
	 */
	dReal 			StdERP		=0.2;
	
	/** \brief Constraint Force Mixing: 
	 * Together with the Error Reduction Parameter, this parameter defines the way the simulation deals with constraints that are not completely matched. This parameter basically defines the 
	 * "spongyness" and "springyness" of a joint (a joint might also be a collision point between two bodies). A higher CFM genrally results in a more stable simulation (less oscillations). 
	 * A lower value, however, reduces the error in joints. 
	 * According to the ode manual, the value should be in the range betwenn 10e-9 and 1. The default value is 10e-5 for single precision and 10e-10 for double precision. 
	 */
	dReal 			StdCFM		=10e-7;

	/*! \brief A map of friction coefficients between materials of a universe. */
	StrFrictionCoefficientMap FrictionCoefficientMap;
	dReal 			StdFrictionCoefficient=1.0;	//!< This is the default friction coefficient. It will be used if the friction coefficient between two materials is not defined. 
	
	/*! \brief A map of coefficients of restitution between materials of a universe. */
	dReal 			StdRestitutionCoefficient=0.01;	//!< This is the default coefficient of restitution (the one that defines how high an object will jump [relative to its drop height] when dropped to the ground). It will be used if the coefficient of restitution between two materials is not defined. 
	
	const unsigned int 	MaxCollisionContacts=10; 	//!< The maximum number of collison points for two bodies. 

	/*! \brief A map of all GeometricBody objects within a universe. */
	StrBodyMap 		BodyMap;
	UCharBfbClientMap	BfbClientMap;

	/*! \brief A map of friction values between materials of a universe. */
	/*! TODO: Implement material dependend friction. */
	//StrFrictionCoefficientMap FrictionCoefficientMap;

	dWorldID 		WorldId		=0;		//!< The ID of the world within the universe (This is the "container" the dynamic bodies will be placed into.).
	dSpaceID 		SpaceId		=0;		//!< The ID of the space within the universe (This is the "container" the collision primitives will be placed into.).
	dJointGroupID 		ContactGroupId	=0;		//!< This is a temporary joint group in which the collision joints are saved during a simulation step. After each simulation step, this contact group will be cleared.
	double 			Time		=0;
	
	/*! \brief Function used by the collision detection engine in order to test the spaces/bodies for collisions among each other.
	 * 	\param data 
	 */
	static void NearCallback(void* data, dGeomID primitive1, dGeomID primitive2);
	
	std::list<boost::shared_ptr<CollisionData>> CollisionDataList={};
	void CreateUniverse();
	void DestroyUniverse();
public:
	Universe(const Universe&) = delete;
	Universe & operator=(const Universe&) = delete;
	
	Universe();
	/*! \brief Class constructor for the Universe class.
	 *
	 *	A Universe holds all objects of a simulation and hence it holds all
	 *	important data of the current state of the physics simulation. It manages
	 *	the time, lists of objects, joints and material friction values.
	 */
	
	/*! \brief Class destructor */
	~Universe();
	
	void Clear();
	
	/*! \brief Let the dynamics and the collision engines simulate the object interactions for a certain period,
	 * 	\param deltaT the time the simulation shall run. The value must be positive.
	 */
	void Simulate(double deltaT);
	//void Simulate(boost::posix_time::time_duration deltaT);
	void SimulateUntill(double absTime);
	
	/*! \brief Draw all the bodies defined in the universe (works only if the visualization is active).	 */
	void Draw();

	/*! \brief Add a rigid body to the simulation
	 *	\param name The name of the GeometricBody object to add to the simulation
	 */
	 boost::shared_ptr<Bodies::RigidBody> AddRigidBody(	std::string name,
							dReal mass=1,
							dRealMatrix3 inertiaTensor=boost::numeric::ublas::identity_matrix<dReal>(3),
							dRealVector3 initialPosition=boost::assign::list_of<dReal>(0)(0)(0),
							dRealMatrix3 initialRotation=boost::numeric::ublas::identity_matrix<dReal>(3),
							dRealVector3 relMassOffset=boost::assign::list_of<dReal>(0)(0)(0),
							std::string material="default",
							std::bitset<32> categoryBits=0,
							std::bitset<32> collideBits=0);

	 boost::shared_ptr<Bodies::StaticBody> AddStaticBody(	std::string name,
							dRealVector3 initialPosition=boost::assign::list_of<dReal>(0)(0)(0),
							dRealMatrix3 initialRotation=boost::numeric::ublas::identity_matrix<dReal>(3),
							std::string material="default",
							std::bitset<32> categoryBits=0,
							std::bitset<32> collideBits=0);
	 
	 boost::shared_ptr<Bodies::StaticBody> AddStaticBody(	std::string name,
							dReal mass,
							dRealMatrix3 inertiaTensor=boost::numeric::ublas::identity_matrix<dReal>(3),
							dRealVector3 initialPosition=boost::assign::list_of<dReal>(0)(0)(0),
							dRealMatrix3 initialRotation=boost::numeric::ublas::identity_matrix<dReal>(3),
							dRealVector3 relMassOffset=boost::assign::list_of<dReal>(0)(0)(0),
							std::string material="default",
							std::bitset<32> categoryBits=0,
							std::bitset<32> collideBits=0);
	 
	void DeleteBody(std::string name);
	 
	boost::shared_ptr<BioFlexRotatory> AddBioFlexRotatory(	unsigned char bioFlexBusId,
								dRealVector3 anchor,
								dRealVector3 axis,
								boost::shared_ptr<Bodies::BaseBody> bodyInput,
								boost::shared_ptr<Bodies::BaseBody> bodyOutput,
								dReal initialOutputAngle,
								dReal springConstant,
								dReal dampingConstant);
	
	boost::shared_ptr<Imu> AttachImuToBody(	unsigned char bioFlexBusId, 
						std::string nameOfBody,
						dRealVector3  offsetPosition=boost::assign::list_of(0)(0)(0), 
						dRealMatrix3  offsetRotation=boost::numeric::ublas::identity_matrix<dReal>(3));

	boost::shared_ptr<PressureSensor> AttachPressureSensorToBody(	unsigned char bioFlexBusId, 
									std::string nameOfBody,
									dRealVector3  offsetPosition=boost::assign::list_of(0)(0)(0), 
									dRealMatrix3  offsetRotation=boost::numeric::ublas::identity_matrix<dReal>(3));
	
	/*! \brief Get a rigid body object by its unique name.
	 *	Universe::GetRigidBody() searches for the object identified by its
	 *	object name
	 *	\param sName The unique object name of the RigidBody object
	 * 	\return a shared pointer to the object
	 */
	boost::shared_ptr<Bodies::BaseBody> GetBody(std::string name);
	
	/*! \brief Get a BioFlex rotatory drive by its unique ID.
	 *	Universe::GetBioFlexRotatory() searches for the drive identified by its
	 *	communication ID
	 *	\param sName The unique object name of the BioFlex rotatory drive object
	 * 	\return a shared pointer to the object
	 */
	boost::shared_ptr<BfbClient> GetBfbClient(unsigned char busId);
	
	/*! \brief Get the ID of the world inside this Universe
	 *	This should be used with care! If you create objects directly - without using the methods of the universe - you can't use the benefits. For example, the universe will not trigger the drawing process of the object. 
	 *	\return the ID of the world of type dWorldID.
	 */
	dWorldID GetWorldId();

	/*! \brief Get the ID of the space inside this Universe
	 *	This should be used with care! If you create objects directly - without using the methods of the universe - you can't use the benefits. For example, the universe will not trigger the drawing process of the object. 
	 *	\return the ID of the space of type dSpaceID.
	 */
	dSpaceID GetSpaceId();

	/*! \brief Get the simulation time since the beginning of the universe (in seconds).
	 *	\return The time since the beginning of the universe in seconds.
	 */
	dReal GetTime();
	
	/*! \brief Get the simulation frequency (in Hertz). 
	 *	\return the simulation frequency
	 */	
	dReal GetSimFrequency();

	/*! \brief Set the simulation frequency (in Hertz). 
	 * 	\param newSimFrequ new simulation frequency
	 */
	void SetSimFrequency(dReal newSimFrequ);

	/*! \brief Get the standard friction coefficient 
	 *	Since the friction depends on the combination of two materials, probably, some combinations won't be specified. 
	 * 	For those combinations, the standard friction coefficient will be used. 
	 * 	\return the standard friction coefficient
	 */
	dReal GetStdFrictionCoefficient();

	/*! \brief Set the standard friction coefficient 
	 *	Since the friction depends on the combination of two materials, probably, some combinations won't be specified. 
	 * 	For those combinations, the standard friction coefficient will be used. 
	 * 	\param newStdFrictionCoefficient the standard friction coefficient
	 */
	void SetStdFrictionCoefficient(dReal newStdFrictionCoefficient);

	/*! \brief Get the standard Error reduction parameter
	 *	The error reduction parameter is used to calculate the "punishment force" exerted on two bodies that are linked with a joint whose onsets have drifted apart. 
	 * 	\return the standard error reduction parameter
	 */
	dReal GetStdERP();

	/*! \brief Set the standard Error reduction parameter
	 *	The error reduction parameter is used to calculate the "punishment force" exerted on two bodies that are linked with a joint whose onsets have drifted apart. 
	 * 	\param newStdERP the standard error reduction parameter
	 */
	void SetStdERP(dReal newStdERP);
	
	/*! \brief Get the standard constraint force mixing parameter
	 *	The constraint force mixing parameter is used together with the error reduction parameter to specify the behavior of the linked bodies (also those that are linked via a collision joint).
	 * 	A higher value will make the joint constraints weaker.
	 *	\return the standard constraint force mixing parameter
	 */
	dReal GetStdCFM();

	/*! \brief Set the standard constraint force mixing parameter
	 *	The constraint force mixing parameter is used together with the error reduction parameter to specify the behavior of the linked bodies (also those that are linked via a collision joint).
	 * 	A higher value will make the joint constraints weaker.
	 *	\param newStdCFMthe the standard constraint force mixing parameter
	 */
	void SetStdCFM(dReal newStdCFM);

	/*! \brief Get the corresponding name to a body-ID
	 *
	 *	Each RigidBody object represents an ODE Body object. Each Body object
	 *	has an ID which is saved inside the GeometricBody object. To get the name
	 *	of an GeometricBody object the map listing all GeometricBody objects is
	 *	searched for an object with the ID equal to the passed parameter.
	 *
	 *	\param bodyID The ID of the GeometricBody object.
	 *
	 *	\return the name of the RigidBody object
	 */
	std::string GetRigidBodyName(dBodyID bodyID);
	
	/*! \brief Get the name of the material for the RigidBody object identified by it's name.
	 *
	 *	\param hBody The ID of the GeometricBody object for which to get material
	 *		information.
	 *
	 *	\return the name of the GeometricBody object's material
	 */
	std::string GetRigidBodyMaterial(dBodyID body);

	/*! \brief Add/Change a new friction value defining the friction between two material
	 *	types.
	 *
	 *	\param material1 The name of the first material.
	 *	\param material2 The name of the second material.
	 *	\param frictionCoefficient the friction coefficient between material 1 and material 2.
	 */
	void SetFrictionCoefficient(	std::string material1,
					std::string material2,
					dReal frictionCoefficient );

	/*! \brief Get the friction between two materials.
	 *
	 *	Each material combination has a distinctive friction coefficient.	 * 
	 *	The friction for two materials can be looked up in the friction map.
	 *
	 *	\param material1 The name of the first material.
	 *	\param material2 The name of the second material.
	 *
	 *	\return the friction term for the material combination. If the material
	 *		combination is not defined the defualt friction term of the universe
	 *		is retruende.
	 */
	dReal GetFrictionCoefficient(std::string material1, std::string material2);
};
#endif

