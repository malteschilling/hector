//#define dDOUBLE // This variable specifies whether the physics engine should use single or double precision. 
// The variable can be set either here or can be set in the compiler command via the -DdDouble option. 
#ifndef ODEDRAWSTUFF_HPP
#define ODEDRAWSTUFF_HPP
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

	/** If the physics engine should work with double precision, it is convenient to use the visualization also with double precision. 
	* Therefore, all default single precision functions will be overwritten with their double precision equivalents.
	*/ 
	#define dsDrawBox dsDrawBoxD 
	#define dsDrawSphere dsDrawSphereD 
	#define dsDrawTriangle dsDrawTriangleD
	#define dsDrawCylinder dsDrawCylinderD 
	#define dsDrawCapsule dsDrawCapsuleD
	#define dsDrawLine dsDrawLineD
	#define dsDrawConvex dsDrawConvexD 
#endif
