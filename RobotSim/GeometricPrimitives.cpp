// STL includes
#include <functional>
#include <iostream>

// Boost includes
#include <boost/array.hpp>
#include <boost/assign.hpp>
#include <boost/concept_check.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/shared_ptr.hpp>

// Own header files
#include "DataTypes.hpp"
#include "GeometricPrimitives.hpp"
#include "HelperFunctions.hpp"
#include "OdeDrawstuff.hpp"

namespace GeometricPrimitives{
// GeometricPrimitiveBase method definitions
float GeometricPrimitiveBase::global_alpha = 1.0;

	GeometricPrimitiveBase::GeometricPrimitiveBase(doubleArray3 color, std::string material): 
		Color(color), 
		Material(material){
		(void)material;
	};

	GeometricPrimitiveBase::~GeometricPrimitiveBase( void ){
		dGeomDestroy(GeomId);
	};

	dGeomID GeometricPrimitiveBase::GetGeomId( void ){
		return GeomId;
	};

	doubleArray3 GeometricPrimitiveBase::GetColor( void ){
		return Color;
	};

	void GeometricPrimitiveBase::SetColor( double red, double green, double blue ){
		Color[0]=red;
		Color[1]=green;
		Color[2]=blue;
	};

	void GeometricPrimitiveBase::SetColor( doubleArray3 color ){
		Color=color;
	};
	
	std::string GeometricPrimitiveBase::GetMaterial()
	{
		return Material;
	}
	
	void GeometricPrimitiveBase::SetMaterial(std::string newMaterial)
	{
		Material=newMaterial;
	}

	bool GeometricPrimitiveBase::isCollisionFeedbackUsed(){
		return (CollisionFeedbackForwardFunctions.size()>0);
	}
	
	void GeometricPrimitiveBase::RouteCollisionFeedbacksTo(std::function<void (const CollisionFeedback collisionFeedback)> callbackFunction){
		CollisionFeedbackForwardFunctions.push_back(callbackFunction);
	}

	void GeometricPrimitiveBase::AddCollisionFeedback(const CollisionFeedback collisionFeedback){
		for(auto it=CollisionFeedbackForwardFunctions.begin(); it!=CollisionFeedbackForwardFunctions.end(); it++){
			(*it)(collisionFeedback);
		};
	}
	

// Sphere method definitions
	Sphere::Sphere(dReal radius, doubleArray3 color, dSpaceID space, std::string material): GeometricPrimitiveBase(color, material), Radius(radius){
		GeomId=dCreateSphere( space, radius);
		dGeomSetData(GeomId, this);
	};
	void Sphere::Draw(){
		dsSetColorAlpha( Color[0], Color[1], Color[2], global_alpha ); 
		dsDrawSphere (dGeomGetPosition(GetGeomId()),dGeomGetRotation (GetGeomId()), Radius);
	};
	dReal Sphere::GetRadius(){
		return Radius;
	}
	void Sphere::SetRadius(dReal radius){
		Radius=radius;
		dGeomSphereSetRadius(GeomId,radius);
	}

// Box method definitions
	Box::Box(dRealVector3 dimension, doubleArray3 color, dSpaceID space, std::string material): GeometricPrimitiveBase(color, material), Dimension(dimension){
		GeomId=dCreateBox( space, Dimension[0], Dimension[1], Dimension[2] );
		dGeomSetData(GeomId, this);
	};
	void Box::Draw(){
		dsSetColorAlpha( Color[0], Color[1], Color[2], global_alpha ); 
		dVector3 temp;
		dGeomBoxGetLengths(GetGeomId(),temp);
		dsDrawBox(dGeomGetPosition(GetGeomId()),dGeomGetRotation (GetGeomId()), temp);
	};
	dRealVector3 Box::GetDimension(){
		return this->Dimension;
	};
	void Box::SetDimension(dRealVector3 dimension){
		Dimension=dimension;
		dGeomBoxSetLengths(GeomId, Dimension[0], Dimension[1], Dimension[2]);
	};

// Plane method definitions

	dReal norm2(dRealVector3 vector){
		return sqrt(pow(vector[0],2)+pow(vector[1],2)+pow(vector[2],2));
	};
	dReal dot(dRealVector3 vector1, dRealVector3 vector2){
		return vector1[0]*vector2[0]+vector1[1]*vector2[1]+vector1[2]*vector2[2];
	};
	
	Plane::Plane(dRealVector3 normal, dReal distance, doubleArray3 color, dSpaceID space, std::string material): 
			GeometricPrimitiveBase(color, material){
		this->GeomId=dCreatePlane( space, normal[0], normal[1], normal[2], distance);
		//UpdateParameters();
		dGeomSetData(GeomId, this);
	};
	
	void Plane::Draw(){
		//std::cout<<"The draw method for 'plane' is not yet implemented."<< std::endl;
	};
	/*
	void Plane::UpdateParameters(){
		boost::numeric::ublas::unit_vector<dReal> tempVec(3,2);
		auto normal=boost::numeric::ublas::prod(OffsetRotation, tempVec);
		boost::numeric::ublas::vector<dReal> tempOffsetPosition;
		tempOffsetPosition.resize(3);
		tempOffsetPosition(0)=OffsetPosition[0];
		dGeomPlaneSetParams (GeomId, normal(0), normal(1), normal(2), boost::numeric::ublas::inner_prod(tempOffsetPosition, normal));
	
	};
	*/
	
// Capsule method definitions

	Capsule::Capsule(dReal radius, dReal length, doubleArray3 color, dSpaceID space, std::string material): GeometricPrimitiveBase(color, material), Radius(radius), Length(length){
		GeomId=dCreateCapsule ( space, Radius, Length);
		dGeomSetData(GeomId, this);
	};
	void Capsule::Draw(){
		dsSetColorAlpha( Color[0], Color[1], Color[2], global_alpha ); 
		dsDrawCapsule(dGeomGetPosition(GetGeomId()),dGeomGetRotation(GetGeomId()),Length, Radius);
	};
	dReal Capsule::GetRadius(){
		return Radius;
	};
	void Capsule::SetRadius(dReal radius){
		Radius=radius;
		dGeomCapsuleSetParams (GeomId, Radius, Length);
	};
	dReal Capsule::GetLength(){
		return Length;
	};
	void Capsule::SetLength(dReal length){
		Length=length;
		dGeomCapsuleSetParams (GeomId, Radius, Length);
	};

// Cylinder method definitions

	Cylinder::Cylinder(dReal radius, dReal length, doubleArray3 color, dSpaceID space, std::string material): GeometricPrimitiveBase(color, material), Radius(radius), Length(length){
		GeomId=dCreateCylinder ( space, Radius, Length);
		dGeomSetData(GeomId, this);
	};
	void Cylinder::Draw(){
		dsSetColorAlpha( Color[0], Color[1], Color[2], global_alpha ); 
		dsDrawCylinder(dGeomGetPosition(this->GetGeomId()),dGeomGetRotation(GetGeomId()),Length, Radius);
	};
	dReal Cylinder::GetRadius(){
		return Radius;
	};
	void Cylinder::SetRadius(dReal radius){
		Radius=radius;
		dGeomCylinderSetParams (GeomId, Radius, Length);
	};
	dReal Cylinder::GetLength(){
		return Length;
	};
	void Cylinder::SetLength(dReal length){
		Length=length;
		dGeomCylinderSetParams (GeomId, Radius, Length);
	};
	
// TriangleMesh method definitions
	
	TriMesh::TriMesh(std::vector<dRealVector3> vertices, std::vector<std::array<unsigned int,3>> faceIndices, doubleArray3 color, dSpaceID space, std::string material): 
			GeometricPrimitiveBase(color, material), 
			Vertices(vertices), 
			FaceIndices(faceIndices),
			VerticesContinuous(std::vector<dReal>(0)),
			FaceIndicesContinuous(std::vector<unsigned int>(0)),
			TriMeshData(dGeomTriMeshDataCreate()){
		
		VerticesContinuous.resize(3*Vertices.size());
		for(unsigned int vertexNum=0; vertexNum<vertices.size(); vertexNum++){
			for(unsigned int dimNum=0;dimNum<3;dimNum++){
				VerticesContinuous.at(vertexNum*3+dimNum)=Vertices.at(vertexNum).at(dimNum);
			};
		};
		
		FaceIndicesContinuous.resize(3*FaceIndices.size());
		for(unsigned int faceIndNum=0;faceIndNum<faceIndices.size();faceIndNum++){
			for(unsigned int dimNum=0;dimNum<3;dimNum++){
				FaceIndicesContinuous.at(faceIndNum*3+dimNum)=FaceIndices.at(faceIndNum).at(dimNum);
			};
		};

		dGeomTriMeshDataBuildDouble(
					TriMeshData,
					const_cast<const dReal*>(VerticesContinuous.data()), 3*sizeof(VerticesContinuous.at(0)), VerticesContinuous.size(),
					const_cast<const unsigned int*>(FaceIndicesContinuous.data()), FaceIndicesContinuous.size(), 3*sizeof(FaceIndicesContinuous.at(0)));

		this->GeomId=dCreateTriMesh(space, TriMeshData, 0, 0, 0);
		dGeomSetData(GeomId, this);
	};
	
	TriMesh::~TriMesh(){
		dGeomTriMeshDataDestroy(TriMeshData);
	}

	
	void TriMesh::Draw(){
		dsSetColorAlpha( Color[0], Color[1], Color[2], global_alpha ); 
		const dReal* tempPos=dGeomGetPosition(GetGeomId());
		const dReal* tempRot= dGeomGetRotation(GetGeomId());
		for(unsigned int faceIndNum=0;faceIndNum < ( this->FaceIndices.size() ); faceIndNum++){
			dsDrawTriangle(tempPos,
				       tempRot, 
				       Vertices.at( FaceIndices.at(faceIndNum).at(0) ).data(),
				       Vertices.at( FaceIndices.at(faceIndNum).at(1) ).data(),
				       Vertices.at( FaceIndices.at(faceIndNum).at(2) ).data(),
				       true);
		}
	};

	Heightfield::Heightfield(dReal xLength, dReal yLength, boost::numeric::ublas::matrix<dReal> zData, bool tile, doubleArray3 color, dSpaceID space, std::string material):
			GeometricPrimitiveBase(color, material), 
			HeightfieldData(dGeomHeightfieldDataCreate()),
			XLength(xLength),
			YLength(yLength),
			ZData(zData),
			XNumOfSamples(zData.size1()),
			YNumOfSamples(zData.size2()),
			XDelta(xLength/(XNumOfSamples-1)),
			YDelta(yLength/(YNumOfSamples-1)),
			IsTiled(tile)
		{
		dReal temp[zData.size1()*zData.size2()];
		unsigned int ind=0;
		for(unsigned int j=0;j<zData.size2();j++){
			for(unsigned int i=0;i<zData.size1();i++){
				temp[ind]=zData(XNumOfSamples-i-1,j);//
				ind++;
			}
		};
		dGeomHeightfieldDataBuildDouble(
			HeightfieldData,
			temp,
			true,
			xLength,yLength,
			XNumOfSamples,YNumOfSamples,
			1, // scaling factor for z-dimension
			0, // z-offset
			1, // thickness
			tile); // should the heightfield  be infintively tiled?
		this->GeomId=dCreateHeightfield(space, HeightfieldData, true); //The last parameter decides whether the geometry should be placeable
		
		auto tempRot=HelperFunctions::RotateAroundAxis(boost::math::constants::pi< double >()/2, boost::assign::list_of<dReal>(1)(0)(0));
		dReal tempArray[12];
		HelperFunctions::DRealMatrix3ToDRealArray12(tempRot,tempArray);
		dGeomSetRotation (GeomId, tempArray);
		
		
		dGeomSetData(GeomId, this);
	}
	
	Heightfield::~Heightfield(){
		dGeomHeightfieldDataDestroy(HeightfieldData);
	}

	
	// On demand of Marc Otto, the following lines are used to change the color of the triangles the heightfield consists of.
	double computeIntensity(double relativeValue){
		if(fabs(relativeValue)<=0.15){
			return 1;
		}else if(fabs(relativeValue)<0.45){
			return 1-3*(fabs(relativeValue)-0.15);
			
		};
		return 0;
	};
	
	std::array<double,3> jet(double min, double max, double value){
		double relativeValue=(value-min)/(max-min);
		
		double red=computeIntensity(relativeValue-0.95);
		double green=computeIntensity(relativeValue-0.65);
		double blue=computeIntensity(relativeValue-0.25);
		std::array<double,3> temp={{red,green,blue}};
		return temp;
	};
	//MOtto end
	
	void Heightfield::Draw(){
		dsSetColorAlpha( Color[0], Color[1], Color[2], global_alpha ); 
		
		doubleArray3 a,b,c,d;
		
		// On demand of Marc Otto, the following lines are used to change the color of the triangles the heightfield consists of.
		double maximum=ZData(0,0);
		double minimum=ZData(0,0);
		for(unsigned long int ind1=0; ind1<ZData.size1(); ind1++){
			for(unsigned long int ind2=0; ind2<ZData.size2(); ind2++){
				if(maximum<ZData(ind1, ind2)){
					maximum=ZData(ind1, ind2);
				}else if(minimum>ZData(ind1, ind2)){
					minimum=ZData(ind1, ind2);
				};
			};
		};
		//MOtto end
			
		
		
		for(double xInd=0; xInd<XNumOfSamples-1; xInd++){
			for(double yInd=0; yInd<YNumOfSamples-1; yInd++){
				
				/* The points a,b,c,d are positioned in the following form:
				 *
				 * a------b
				 * |\     |
				 * | \    |
				 * |  \   |
				 * |   \  |
				 * |    \ |
				 * |     \|
				 * c------d
				 * 
				 */ 

				a[0]=XLength/2-xInd*XDelta;
				a[1]=YLength/2-yInd*YDelta;
				a[2]=ZData(xInd,yInd);
				
				b[0]=XLength/2-(xInd+1)*XDelta;
				b[1]=YLength/2-yInd*YDelta;
				b[2]=ZData(xInd+1,yInd);

				c[0]=XLength/2-(xInd)*XDelta;
				c[1]=YLength/2-(yInd+1)*YDelta;
				c[2]=ZData(xInd,yInd+1);
				
				d[0]=XLength/2-(xInd+1)*XDelta;
				d[1]=YLength/2-(yInd+1)*YDelta;
				d[2]=ZData(xInd+1,yInd+1);
				
				dRealMatrix3 absRot=HelperFunctions::DRealArray12ToDRealMatrix3(dGeomGetRotation(GetGeomId()));
				auto correctiveRot=HelperFunctions::RotateAroundAxis(-boost::math::constants::pi< double >()/2, boost::assign::list_of<dReal>(1)(0)(0));
				absRot=boost::numeric::ublas::prod(absRot,correctiveRot);
				double tempRot[12];
				HelperFunctions::DRealMatrix3ToDRealArray12(absRot, tempRot);
				
				// On demand of Marc Otto, the following lines are used to change the color of the triangles the heightfield consists of.
				double tempMax1=a[2];
				double tempMax2;
				if(d[2]>tempMax1){
					tempMax1=d[2];
				};
				tempMax2=tempMax1;
				if(b[2]>tempMax1){
					tempMax1=b[2];
				};
				if(c[2]>tempMax2){
					tempMax2=c[2];
				};
				auto color=jet(minimum, maximum, tempMax1);
				dsSetColorAlpha( color[0], color[1], color[2], global_alpha ); 
				
				//MOtto end
				dsDrawTriangle(dGeomGetPosition(GetGeomId()),
					tempRot, 
					a.data(),
					b.data(),
					d.data(),
					true);
				// On demand of Marc Otto, the following lines are used to change the color of the triangles the heightfield consists of.
				color=jet(minimum, maximum, tempMax2);
				dsSetColorAlpha( color[0], color[1], color[2], global_alpha ); 
				//MOtto end
				dsDrawTriangle(dGeomGetPosition(GetGeomId()),
					tempRot, 
					a.data(),
					d.data(),
					c.data(),
					true);
				
			};
		};
	};

};// end of namespace
