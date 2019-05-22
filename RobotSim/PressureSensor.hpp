#ifndef PRESSURESENSOR_HPP
#define PRESSURESENSOR_HPP

// STL includes
#include <list>
#include <utility>

// Boost includes
#include <boost/assign.hpp>
#include <boost/numeric/ublas/matrix.hpp>

// Own header files
#include "Bodies.hpp"
//#include "CollisionFeedback.hpp"
#include "DataTypes.hpp"
#include "OdeDrawstuff.hpp"
#include "GeometricPrimitives.hpp"
#include "BfbClient.hpp"

class PressureSensor : public BfbClient{
	
	private:
		std::list<CollisionFeedback> CollisionFeedbacks={};
		std::list<dRealVector3> Forces={}; // pressure values of the cells 
		boost::weak_ptr<Bodies::BaseBody> Body;
	public:
		PressureSensor(const PressureSensor&) = delete;
		PressureSensor & operator=(const PressureSensor&) = delete;
		
		PressureSensor(unsigned char bioFlexBusId, boost::weak_ptr<Bodies::BaseBody> body, dRealVector3  offsetPosition=boost::assign::list_of(0)(0)(0), dRealMatrix3  offsetRotation=boost::numeric::ublas::identity_matrix<dReal>(3));

		virtual ~PressureSensor() { };

		void AddCollisionFeedback(const CollisionFeedback collisionFeedback);
		
		dReal GetHighestPressure();

		virtual void PostSimulationStepUpdate(double deltaT); 
		virtual boost::shared_ptr<BfbMessage> ProcessMessage(boost::shared_ptr<const BfbMessage> message);
};

#endif // PRESSURESENSOR_HPP
