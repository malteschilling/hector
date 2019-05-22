#ifndef DATAYTYPES_HPP
#define DATAYTYPES_HPP

// Boost includes
#include <boost/array.hpp>
#include <boost/numeric/ublas/matrix.hpp>

// Own header files
#include "OdeDrawstuff.hpp"

typedef boost::array<double,3> doubleArray3;
typedef boost::array<dReal,3> dRealVector3;
typedef boost::numeric::ublas::c_matrix<dReal,3,3> dRealMatrix3;

struct CollisionFeedback{
	dRealVector3 AbsPosition;
	dRealVector3 Normal;
	dRealVector3 FeedbackForce;
	dRealVector3 FeedbackTorque;
};

#endif 