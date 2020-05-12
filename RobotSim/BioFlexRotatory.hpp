#ifndef BIOFLEXROTATORY_HPP
#define BIOFLEXROTATORY_HPP

// STL includes
#include <iostream>
#include <string>

// Boost includes
#include <boost/weak_ptr.hpp>
#include <boost/date_time.hpp>

// Own header files
#include "BfbClient.hpp"
#include "Bodies.hpp"
#include "Joints.hpp"
#include "OdeDrawstuff.hpp"

class BioFlexRotatory : public Joints::Hinge, public BfbClient
{
private:
	dReal SpringConstant;
	dReal DampingConstant;
	dReal MaxTorque;
	dReal InputSpeed;
	dReal InputAngle;
	dReal OutputAngle;
	dReal Torsion;
	dReal Torque;
	bool Activation;
	double Time=0;
	
	bool Tmc603aError=false;
	bool CommunicationTimeoutError=false;
	bool TorsionMeasurementError=false;
	bool OutputAngleMeasurementError=false; // This corresponds to the OutputPositionMeasurementError.
	bool RotorOvertemperatureError=false;
	bool DriverOvertemperatureError=false;
	bool InputAngleError=false;
	bool OutputAngleError=false;
	bool TorsionError=false;
	bool WatchdogError=false;
	bool OvervoltageError=false;
	bool UndervoltageError=false;
	bool I2cError=true;
	
	unsigned char ResetState=1;

public:

	BioFlexRotatory(		dWorldID worldId,
					unsigned char bioFlexBusId,
					dRealVector3 anchor,
					dRealVector3 axis,
					boost::weak_ptr<Bodies::BaseBody> bodyInput,
					boost::weak_ptr<Bodies::BaseBody> odyOutput,
					dReal initialOutputAngle,
					dReal springConstant,
					dReal dampingConstant);
	
	void Reset();
	
	void PostSimulationStepUpdate(double deltaT); 
	boost::shared_ptr<BfbMessage> ProcessMessage(boost::shared_ptr<const BfbMessage> Message);

	bool GetActivation();
	void SetActivation(bool newActivation);
	
	void SetInputPosition(dReal inputPos);
	
	void SetInputSpeed(dReal inputSpeed);
	dReal GetInputSpeed();
	dReal GetOutputSpeed();
	/*! \brief Get the applied torque
	 *
	 */
	dReal GetTorque( );

	/*! \brief Get the target angle
	 *
	 */
	dReal GetInputAngle( );
	dReal GetTorsionAngle( );
	dReal GetOutputAngle( );

	/*! \brief Set an AMotor joint's angle velocity
	 *
	 *	Joint::SetAngleVelocity() sets the angle velocity for an AMotor joint.
	 *	
	 *	\param nAxis The axis of the AMotor to set the velocity for.
	 *	\param fVelocity the velocity to set for the specified axis.
	 *
	 *	\return one if successful.
	 *	\return zero if SetAngleVelocity() fails.
	 */

	/*! \brief Set an AMotor joint's maximum force.
	 *
	 *	Joint::SetMaximumForce() sets the maximum force of an AMotor joint.
	 *	
	 *	\param nAxis The axis of the AMotor to set the maximum force for.
	 *	\param fForce the maximum force to set for the specified axis.
	 *
	 *	\return one if successful.
	 *	\return zero if SetAngleVelocity() fails.
	 */
	 void SetMaximumTorque(dReal torque);

	/*! \brief Set the damping of the joint
	 *
	 *	\return zero
	 */
	void SetDampingConstant(dReal dampingConstant);

	/*! \brief Get the damping of the joint
	 *
	 *	\return zero
	 */
	dReal GetDampingConstant( );

	/*! \brief Set the spring constant of the joint
	 *
	 *	\return zero
	 */
	void SetSpringConstant(dReal springConstant);

	/*! \brief Get the spring constant of the joint
	 *
	 *	\return zero
	 */
	dReal GetSpringConstant( );
};

#endif
