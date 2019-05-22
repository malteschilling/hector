#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

// Boost includes
#include <boost/assign.hpp>
#include <boost/numeric/ublas/matrix.hpp>

// Own header files
#include "DataTypes.hpp"
#include "OdeDrawstuff.hpp"

namespace HelperFunctions{
	void DRealMatrix3ToDRealArray12(dRealMatrix3 Matrix, dReal TempArray[12]);
	dRealMatrix3 DRealArray12ToDRealMatrix3(const dReal Array[12]);
	dRealVector3 DRealArrayToDRealVector3(dReal Array[3]);
	void DRealVector3ToDRealArray(dRealVector3 Vec, dReal Array[3]);
	dRealMatrix3 RotateAroundAxis(dReal Angle, dRealVector3 Axis);
	dMass CreateDMass(dReal mass, dRealMatrix3 momentOfInertia);
	boost::numeric::ublas::matrix<double> Rot90(boost::numeric::ublas::matrix<double> matrix);
	dRealMatrix3 invertRotationMatrix(const dRealMatrix3 input);

	
	
	
};
#endif
