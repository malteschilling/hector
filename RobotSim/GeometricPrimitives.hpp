#ifndef GEOMETRICPRIMITIVES_H
#define GEOMETRICPRIMITIVES_H

// STL includes
#include <array>
#include <functional>

// Boost includes
#include <boost/array.hpp>
#include <boost/assign.hpp>
#include <boost/concept_check.hpp>
#include <boost/numeric/ublas/matrix.hpp>

// Own header files
//#include "CollisionFeedback.hpp"
#include "DataTypes.hpp"
#include "OdeDrawstuff.hpp"


namespace GeometricPrimitives{

class GeometricPrimitiveBase
{
	protected:
		dGeomID 	GeomId=0;
		doubleArray3 	Color;		//!< Object color in RGB. This is only relevant for the visualization using the integrated 3D engine. 
		std::string	Material;
			/*! \brief Simple ExtendedPrimitive class constructor
		 *
		 *	The simple class constructor creates an extended primitive with no body associated with this geometry. 
		 * 	A body can later be associated with this geometry by using the member function associateBody().
		 *
		 *	\param GeometryID The unique identifier of the geometry.
		 *	\param OffsetPosition The local offset position of a geometry from its body
		 */
		GeometricPrimitiveBase(doubleArray3 color, std::string material="default");
		
		std::vector<std::function<void (const CollisionFeedback collisionFeedback)>> CollisionFeedbackForwardFunctions={};
		
	public:
		static float global_alpha;	
		GeometricPrimitiveBase(const GeometricPrimitiveBase&) = delete;
		GeometricPrimitiveBase & operator=(const GeometricPrimitiveBase&) = delete;
 
		//! \brief The class destructor
		virtual ~GeometricPrimitiveBase( );
		
		/*! \brief Get the ID of the geometry inside the extended primitive
		 *
		 *	\return the ID of the geometry of type \a dGeomID.
		 */
		dGeomID GetGeomId( );
		
		/*! \brief Get the color of the extended primitive.
		 *
		 *	\return an Array of type double and a size of three where the index
		 *		corresponds to RGB in ascending order (0: red, 1: green, 2: blue)
		 */
		doubleArray3 GetColor( void );

		/*! \brief Set the color of the extended primitive.
		 *
		 *	\param Red	the amount of red in the additive mixed color. A value between 0 to 1.
		 *	\param Green the amount of green in the additive mixed color. A value between 0 to 1.
		 *	\param Blue the amount of blue in the additive mixed color. A value between 0 to 1.
		 */
		void SetColor(	double red,
				double green,
				double blue );

		/*! \brief Set the color of the extended primitive.
		 *
		 *	\param Color Array of type double and a size of three where the index corresponds to RGB in ascending order (0: red, 1: green, 2: blue)
		 */
		void SetColor( doubleArray3 color );
		
		std::string GetMaterial();
		
		void SetMaterial(std::string newMaterial);
		
		virtual void Draw()=0;
		
		bool isCollisionFeedbackUsed();
		void AddCollisionFeedback(const CollisionFeedback collisionFeedback);
		
		void RouteCollisionFeedbacksTo(std::function<void (const CollisionFeedback collisionFeedback)> callbackFunction);
		
};
	
	
	
class Sphere : public GeometricPrimitiveBase
{
	private:
		dReal Radius;
	public:
		Sphere(const GeometricPrimitiveBase&) = delete;
		Sphere & operator=(const Sphere&) = delete;
 
		
		Sphere(dReal radius, doubleArray3 color, dSpaceID space=0, std::string material="default");
		dReal GetRadius();
		void SetRadius(dReal radius);
		void Draw();
};

class Box : public GeometricPrimitiveBase
{
	private:
		dRealVector3 Dimension;
	public:
		Box(const Box&) = delete;
		Box & operator=(const Box&) = delete;
		
		Box(dRealVector3 dimension, doubleArray3 color, dSpaceID space=0, std::string material="default");
		dRealVector3  GetDimension();
		void SetDimension(dRealVector3 dimension);
		void Draw();
};

class Plane : public GeometricPrimitiveBase
{
	private:
		void UpdateParameters();
		void SetOffsetPosition(dRealVector3 offsetPosition);
		void SetOffsetRotation(dRealMatrix3 offsetRotation);
	public:
		Plane(const Plane&) = delete;
		Plane & operator=(const Plane&) = delete;
		
		Plane(dRealVector3 normal, dReal distance, doubleArray3 color, dSpaceID space=0, std::string material="default");
		void Draw();
};

class Capsule : public GeometricPrimitiveBase
{
	private:
		dReal Radius;
		dReal Length;
	public:
		Capsule(const Capsule&) = delete;
		Capsule & operator=(const Capsule&) = delete;
		
		Capsule(dReal radius, dReal length, doubleArray3 color, dSpaceID space=0, std::string material="default");
		dReal GetRadius();
		void SetRadius(dReal radius);
		dReal GetLength();
		void SetLength(dReal length);
		void Draw();
};

class Cylinder : public GeometricPrimitiveBase
{
	private:
		dReal Radius;
		dReal Length;
	public:
		Cylinder(const Cylinder&) = delete;
		Cylinder & operator=(const Cylinder&) = delete;
		
		Cylinder(dReal radius, dReal length, doubleArray3 color, dSpaceID space=0, std::string material="default");
		dReal GetRadius();
		void SetRadius(dReal radius);
		dReal GetLength();
		void SetLength(dReal length);
		void Draw();
};


class TriMesh : public GeometricPrimitiveBase
{
	private:
		std::vector<dRealVector3> Vertices;
		std::vector<std::array<unsigned int,3>> FaceIndices;
		std::vector<dReal> VerticesContinuous; 
		std::vector<unsigned int> FaceIndicesContinuous; 
		dTriMeshDataID TriMeshData;
	public:
		TriMesh(const TriMesh&) = delete;
		TriMesh & operator=(const TriMesh&) = delete;
		
		TriMesh(std::vector<dRealVector3> vertices, std::vector<std::array<unsigned int,3>> faceIndices, doubleArray3 color, dSpaceID space=0, std::string material="default");
		virtual ~TriMesh();
		void Draw();
};

class Heightfield : public GeometricPrimitiveBase
{
	private:
		dHeightfieldDataID HeightfieldData;
		double XLength;
		double YLength;
		boost::numeric::ublas::matrix<dReal> ZData;
		unsigned int XNumOfSamples;
		unsigned int YNumOfSamples;
		double XDelta;
		double YDelta;
		bool IsTiled;
	public:
		Heightfield(const Heightfield&) = delete;
		Heightfield & operator=(const Heightfield&) = delete;
		
		Heightfield(dReal xLength, dReal yLength, boost::numeric::ublas::matrix<dReal> zData, bool tile, doubleArray3 color, dSpaceID space=0, std::string material="default");
		virtual ~Heightfield();
		void Draw();
};

}; // End of namespace
#endif
