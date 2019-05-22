// Boost includes
#include <boost/concept_check.hpp>

// Own header files
#include "DataTypes.hpp"
#include "HelperFunctions.hpp"
#include "OdeDrawstuff.hpp"



namespace HelperFunctions{
	
	void DRealMatrix3ToDRealArray12(dRealMatrix3 Matrix, dReal TempArray[12]){
		for(int row=0;row<3;row++){
			for(int col=0;col<3;col++){
				TempArray[row*4+col]=Matrix(row,col);
			};
			TempArray[row*4+3]=0;
		};
	};

	dRealMatrix3 DRealArray12ToDRealMatrix3(const dReal Array[12]){
		dRealMatrix3 TempMatrix;
		for(unsigned int i=0;i<3;i++){
			for(unsigned int j=0;j<3;j++){
				TempMatrix(i,j)=Array[i*4+j];
			};
		};
		return TempMatrix;
	};
	
	dRealVector3 DRealArrayToDRealVector3(dReal Array[12]){
		dRealVector3 Vec;
		for(unsigned int i=0;i<3;i++){Vec[i]=Array[i];};
		return Vec;
	};
	void DRealVector3ToDRealArray(dRealVector3 Vec, dReal Array[12]){
		for(unsigned int i=0;i<3;i++){Array[i]=Vec[i];};
	};
	
	/* Copied from wikipedia: http://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
	parameters: a: angle to rotate, rotec vector of rotation axis
	*/
	dRealMatrix3 RotateAroundAxis(dReal Angle, dRealVector3 Axis) {
		dRealMatrix3 RotationMatrix;
		double CosTheta = cos(Angle);
		double SinTheta = sin(Angle);
		dReal ux=Axis[0];
		dReal uy=Axis[1];
		dReal uz=Axis[2];
		RotationMatrix(0,0) = CosTheta+pow(ux,2)*(1-CosTheta);
		RotationMatrix(0,1) = ux*uy*(1-CosTheta)-uz*SinTheta;
		RotationMatrix(0,2) = ux*uz*(1-CosTheta)+uy*SinTheta;
		RotationMatrix(1,0) = uy*ux*(1-CosTheta)+uz*SinTheta;
		RotationMatrix(1,1) = CosTheta+pow(uy,2)*(1-CosTheta);
		RotationMatrix(1,2) = uy*uz*(1-CosTheta)-ux*SinTheta;
		RotationMatrix(2,0) = uz*ux*(1-CosTheta)-uy*SinTheta;
		RotationMatrix(2,1) = uz*uy*(1-CosTheta)+ux*SinTheta;
		RotationMatrix(2,2) = CosTheta+pow(uz,2)*(1-CosTheta);
		return RotationMatrix;
	}
	
	dMass CreateDMass(dReal mass, dRealMatrix3 momentOfInertia){
		dMass tempMass;
		dMassSetParameters (	&tempMass, 
					mass,
					0, 0, 0,
					momentOfInertia(0,0), momentOfInertia(1,1), momentOfInertia(2,2),
					momentOfInertia(0,1), momentOfInertia(0,2), momentOfInertia(1,2));
	return tempMass;
	};
	
	boost::numeric::ublas::matrix<double> Rot90(boost::numeric::ublas::matrix<double> matrix){
		boost::numeric::ublas::matrix<double> tempMatrix(matrix.size2(),matrix.size1());
		for(unsigned int i=0;i<tempMatrix.size1();i++){
			for(unsigned int j=0;j<tempMatrix.size2();j++){
				tempMatrix(i,j)=matrix(j,matrix.size2()-i-1);
			}
		}
		return tempMatrix;
	}
	
	// This function can be used to invert rotation matrices ( matrices whose determinant is 1 ).
	dRealMatrix3 invertRotationMatrix(const dRealMatrix3 input){
		dRealMatrix3 output;
		for(unsigned int r=0; r<3; r++){
			for (unsigned int c=0; c<3; c++){
				output(c,r)=input(r,c);
			};
		};
		return output;
	}
	

}
