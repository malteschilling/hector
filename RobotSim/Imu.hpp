#ifndef IMU_HPP 
#define IMU_HPP

// STL includes
#include <list>

// Boost includes
#include <boost/assign.hpp>
#include <boost/numeric/ublas/matrix.hpp>

// Own header files
#include "DataTypes.hpp"
#include "OdeDrawstuff.hpp"
#include "BfbClient.hpp"
#include "Bodies.hpp"

class Imu : public BfbClient
{
	private:
		boost::weak_ptr<Bodies::BaseBody> Body;
		dRealVector3 	OffsetPosition;
		dRealMatrix3 	OffsetRotation;
		dRealMatrix3 	InvOffsetRotation;
		
		dRealVector3 	RelAcceleration=boost::assign::list_of(0)(0)(0);
		dRealVector3	Velocity=boost::assign::list_of(0)(0)(0);
		dRealVector3	LastVelocity=boost::assign::list_of(0)(0)(0);
		
	public:
		Imu(	unsigned char bioFlexBusId, 
			boost::weak_ptr<Bodies::BaseBody> body,
			dRealVector3  offsetPosition=boost::assign::list_of(0)(0)(0), 
			dRealMatrix3  offsetRotation=boost::numeric::ublas::identity_matrix<dReal>(3));
		
		virtual ~Imu() { };

		dRealVector3 GetPosition();
		dRealMatrix3 GetRotation();
		dRealVector3 GetRelMagneticField();
		dRealVector3 GetRelAcceleration();
		
		virtual boost::shared_ptr<BfbMessage> ProcessMessage(boost::shared_ptr<const BfbMessage> Message);
		
		virtual void PostSimulationStepUpdate(double deltaT);
};

#endif // IMU_HPP
