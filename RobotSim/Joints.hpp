#ifndef JOINT_H
#define JOINT_H

// STL includes
#include <string>

// Boost includes
#include <boost/shared_ptr.hpp>

// Own header files
#include "Bodies.hpp"
#include "OdeDrawstuff.hpp"


//#define dDOUBLE		//!< ODE double precision
namespace Joints{
	
class BaseJoint{
	protected:
		dJointID JointId=0;		//!< Joint identifier
		std::string Name;	//!< Joint name
		boost::weak_ptr<Bodies::BaseBody> Body1;
		boost::weak_ptr<Bodies::BaseBody> Body2;
		
		
	public:
		BaseJoint(const BaseJoint&) = delete;
		BaseJoint & operator=(const BaseJoint&) = delete;
		
		BaseJoint(std::string name, boost::weak_ptr<Bodies::BaseBody> body1, boost::weak_ptr<Bodies::BaseBody> body2);
		virtual ~BaseJoint();
		
		/*! \brief Get the name of the joint.
		 *
		 *	\return the name of the joint.
		 */
		std::string GetName( );

		/*! \brief Get the unique identifier of the joint.
		 *
		 *	\return the unique identifier of the joint.
		 */
		dJointID GetJointId( );
		
		void Attach(boost::weak_ptr<Bodies::BaseBody> body1, boost::weak_ptr<Bodies::BaseBody> body2);
		bool IsAttached();
		//bool IsAttachedToBodies();
		boost::weak_ptr<Bodies::BaseBody> GetBody1();
		boost::weak_ptr<Bodies::BaseBody> GetBody2();
};
	
/*! \brief A short class description
 *
 *	A Detail description of Friction
 */
class Hinge:public BaseJoint{
	protected:
		void SetAnchor(dRealVector3 anchor);
		dRealVector3 GetAnchor( );
		
		void SetAxis(dRealVector3 axis);
		dRealVector3 GetAxis( );
		dReal AngularOffset;
		
	public:
		Hinge(const Hinge&) = delete;
		Hinge & operator=(const Hinge&) = delete;
		
		/*! \brief Constructor of the Hinge class.
		 *
		 *
		 */
		Hinge(	dWorldID worldId,
				std::string name,
				dRealVector3 anchor,
				dRealVector3 axis,
				boost::weak_ptr<Bodies::BaseBody> body1,
				boost::weak_ptr<Bodies::BaseBody> body2,
				dReal angularOffset);
		
		dReal GetAngle();
};
};
#endif
