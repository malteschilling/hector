// STL includes
#include <array>
#include <fstream>
#include <stdlib.h>
#include <istream>
#include <iterator>
#include <sstream>
#include <set>
#include <limits>

// Boost includes
#include <boost/algorithm/string.hpp>
#include <boost/assign.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/shared_ptr.hpp>

// External library files
#include <pugixml.hpp>

// Own header files
#include "Bodies.hpp"
#include "DataTypes.hpp"
#include "HelperFunctions.hpp"
#include "OdeDrawstuff.hpp"
#include "Universe.hpp"
#include "GeometryXmlParser.hpp"


void print(pugi::xml_node node,unsigned int maxdepth=2, std::string indentation="")
{
	if(std::string(node.name()).size()!=0){
		std::cout<<indentation<<node.name()<<std::endl;
		if(node.attributes_begin()!=node.attributes_end()){
			std::cout<<indentation<<"    "<<"Attributes:"<<std::endl;
			for(auto it=node.attributes_begin();it!=node.attributes_end();it++){
				std::cout<<indentation<<"        "<<it->name()<<": "<<it->value()<<std::endl;
			};
		}
		if(node.begin()!=node.end()){
			for(auto it=node.begin();it!=node.end();it++){
				print(*it,maxdepth-1,indentation+"    ");
			};
		}else if(std::string(node.value()).size()!=0){
			std::cout<<indentation<<"        "<<"Value: "<<node.value()<<std::endl;
		};
	}else{
		std::cout<<indentation<<"        "<<node.value()<<std::endl;
	};
};


template <typename Type>
Type getValue(pugi::xml_node node, std::string name){
	std::string tempStr=node.child_value(name.c_str());
	if(tempStr.size()==0){
		for(auto it=node.begin();it!=node.end();it++){
			if(boost::to_lower_copy<std::string>(it->name())==boost::to_lower_copy<std::string>(name)){
				tempStr=it->value();
				break;
			}
		}
	}
	
	if(tempStr.size()==0){
		tempStr=node.attribute(name.c_str()).value();
	}
	
	if(tempStr.size()==0){
		for(auto it=node.attributes_begin();it!=node.attributes_end();it++){
			if(boost::to_lower_copy<std::string>(it->name())==boost::to_lower_copy<std::string>(name)){
				tempStr=it->value();
				break;
			}
		}
	}
	
	if(tempStr.size()==0){
		throw std::invalid_argument("No value found for entry with name '"+ name + "'.");
	}
	//std::cout<<"about to convert "+ name<<std::endl;
	return boost::lexical_cast<Type>(tempStr);
};

template <>
unsigned char getValue<unsigned char>(pugi::xml_node node, std::string name){
	return boost::numeric_cast<unsigned char>(getValue<unsigned int>(node, name));
};

template <>
signed char getValue<signed char>(pugi::xml_node node, std::string name){
	return boost::numeric_cast<signed char>(getValue<signed int>(node, name));
};

template <>
bool getValue<bool>(pugi::xml_node node, std::string name){
	std::string tempStr=getValue<std::string>(node, name);
	if(boost::to_lower_copy(tempStr)=="true"){
		return true;
	}else if(boost::to_lower_copy(tempStr)=="false"){
		throw std::invalid_argument("Trying to convert an entry to bool, but it turns out it is impossible to convert '"+ tempStr + "' to bool.");
	};
	return false;
};

template <typename Type>
Type getValue(pugi::xml_node node, std::string name, Type defaultValue){
	Type tempValue;
	try{
		tempValue=getValue<Type>(node, name);
	}catch(std::invalid_argument &err){
		tempValue=defaultValue;
	};
	return tempValue;
};

doubleArray3 ProcessArray3Node(pugi::xml_node node){
	doubleArray3 tempArray=boost::assign::list_of<dReal>(0)(0)(0);
	for(auto it=node.begin(); it!=node.end(); it++){
		if(std::string(it->name())=="Value"){
			auto index=getValue<unsigned int>(*it, "Index");
			tempArray[index]=it->text().as_double();
		}
	};
	return tempArray;
}

dRealVector3 ProcessVector3Node(pugi::xml_node node){
	auto tempArray=ProcessArray3Node(node);
	dRealVector3 tempVec=boost::assign::list_of<dReal>(0)(0)(0);
	for(unsigned int i=0; i<tempVec.size() && i<tempArray.size() ; i++){
		tempVec[i]=tempArray.at(i);
	};
	return tempVec;
}

dRealMatrix3 ProcessMatrix3Node(pugi::xml_node node){
	dRealMatrix3 tempMatrix=boost::numeric::ublas::identity_matrix<dReal>(3);
	for(auto it=node.begin(); it!=node.end(); it++){
		if(std::string(it->name())=="Value"){
			auto row=getValue<unsigned int>(*it, "Row");
			auto col=getValue<unsigned int>(*it, "Col");
			tempMatrix(row, col)=it->text().as_double();
		}
	};
	return tempMatrix;
}

boost::numeric::ublas::matrix<double> ProcessMatrixNode(pugi::xml_node node){
	boost::numeric::ublas::matrix<double> tempMatrix(1,1);
	for(auto it=node.begin(); it!=node.end(); it++){
		if(std::string(it->name())=="Value"){
			auto row=getValue<unsigned int>(*it, "Row");
			auto col=getValue<unsigned int>(*it, "Col");
			if(row>=tempMatrix.size1()){
				tempMatrix.resize(row+1,tempMatrix.size2());
			};
			if(col>=tempMatrix.size2()){
				tempMatrix.resize(tempMatrix.size1(),col+1);
			};
			tempMatrix(row, col)=it->text().as_double();
		}
	};
	return tempMatrix;
}


dRealMatrix3 ProcessTensor3Node(pugi::xml_node node){
	return ProcessMatrix3Node(node);
}

std::vector<dRealVector3> ProcessVerticesNode(pugi::xml_node node){
	std::vector<dRealVector3> vertices;
	vertices.resize(std::distance(node.begin(), node.end()));
	for(auto it=node.begin(); it!=node.end(); it++){
		if(std::string(it->name())=="Vertex"){
			auto index=getValue<unsigned int>(*it, "Index");
			vertices[index]=ProcessVector3Node(it->child("Vector3"));
		}
	}
	return vertices;
}

std::vector<std::array<unsigned int, 3>> ProcessFacesNode(pugi::xml_node node){
	std::vector<std::array<unsigned int, 3>> faces;
	faces.resize(std::distance(node.begin(), node.end()));
	for(auto it=node.begin(); it!=node.end(); it++){
		if(std::string(it->name())=="Face"){
			auto index=getValue<unsigned int>(*it, "Index");
			auto tempArray=ProcessArray3Node(it->child("Array3"));
			for(unsigned int j=0; j<3; j++){
				faces[index][j]=tempArray.at(j);
			};
		}
	}
	return faces;
}

void ProcessGeometricPrimitiveNode(pugi::xml_node node, boost::shared_ptr<Bodies::BaseBody > body, std::string defaultName){
	pugi::xml_node tempNode;
	
	std::string type=getValue<std::string>(node, "Type", "Sphere");
	std::string name=getValue<std::string>(node, "Name", defaultName);

	
	dRealVector3 offsetPosition=boost::assign::list_of<dReal>(0)(0)(0);
	tempNode=node.child("OffsetPosition").child("Vector3");
	if(tempNode){
		offsetPosition=ProcessVector3Node(tempNode);
	};
	
	dRealMatrix3 offsetRotation=boost::numeric::ublas::identity_matrix<dReal>(3);
	tempNode=node.child("OffsetRotation").child("Matrix3");
	if(tempNode){
		offsetRotation=ProcessMatrix3Node(tempNode);
	};
	
	doubleArray3 color=boost::assign::list_of<double>(0)(0)(0);
	tempNode=node.child("Color");
	if(tempNode){
		std::array<std::string,3> colorInitials={{"R","G","B"}};
		for(unsigned int i=0; i<3; i++){
			color[i]=getValue<double>(tempNode, colorInitials[i]);
		};
	};
	
	if(type=="Box"){
		dRealVector3 dimension=boost::assign::list_of<double>(1)(1)(1);
		tempNode=node.child("Dimension").child("Array3");
		if(tempNode){
			dimension=ProcessArray3Node(tempNode);
		};
		body->AddBox( dimension, offsetPosition, offsetRotation, color );
	}else if(type=="Sphere" || type=="Capsule"|| type=="Cylinder"){
		dReal radius=getValue<dReal>(node, "Radius", 0.5);
		if(type=="Sphere"){
			body->AddSphere(radius, offsetPosition, offsetRotation, color);
		}else{
			dReal length=getValue<dReal>(node, "Length", 1);
			if(type=="Capsule"){
				body->AddCapsule( radius, length, offsetPosition, offsetRotation, color );
			}else{
				body->AddCylinder( radius, length, offsetPosition, offsetRotation, color );
			};
		};
	}else if(type=="TriMesh"){
		std::vector<dRealVector3> vertices;
		tempNode=node.child("Vertices");
		if(tempNode){
			vertices=ProcessVerticesNode(tempNode);
		};
		
		std::vector<std::array<unsigned int,3>> faces;
		tempNode=node.child("Faces");
		if(tempNode){
			faces=ProcessFacesNode(tempNode);
		};
		
		if(vertices.size()>0 && faces.size()>0){
			body->AddTriMesh( vertices, faces, offsetPosition, offsetRotation, color);
		};
	}else if(type=="Plane"){
		// This works only for static bodies. Therefore, the base body must be casted first into a static body. 
		Bodies::StaticBody* staticBody = dynamic_cast<Bodies::StaticBody*>(body.get());
		if(staticBody){
			staticBody->AddPlane(offsetPosition, offsetRotation, color);
		};
	}else if(type=="Heightfield"){
		dReal xLength=getValue<dReal>(node, "XLength", 1);
		dReal yLength=getValue<dReal>(node, "YLength", 1);
		boost::numeric::ublas::matrix<dReal> zData;
		tempNode=node.child("ZData").child("Matrix");
		if(tempNode){
			zData=ProcessMatrixNode(tempNode);
		};		
		
		
		/* The commented block below was used to create a true Heightfield, but as tests have shown, heightfields are not well implemented (Sometimes, objects fall through the seams and such kind of things...). Therefore, the native heightfield has been replaced by a trimesh. The disadvantage of using trimeshes is that they are computationally more expensive. Therefore, you should divide a big "heightfield" into multiple smaller "heightfields" in order to reduce the number of collision checks as soon as the "heightfield's" bounding box overlaps with the bounding box of another object.
		bool tile=getValue<bool>(node, "tile", false);
		
		body->AddHeightfield(	xLength, 
					yLength, 
					zData, 
					tile, 
					offsetPosition, 
					offsetRotation, 
					color);
		*/

		// The following code is used to replace the native heigthfield with a trimesh.
		std::vector<double> xTicks(zData.size1(),0);
		for(unsigned int i=0; i<zData.size1(); i++){
			xTicks[i]=xLength/2-i*xLength/(zData.size1()-1);
		};
		std::vector<double> yTicks(zData.size2(),0);
		for(unsigned int i=0; i<zData.size2(); i++){
			yTicks[i]=yLength/2-i*yLength/(zData.size2()-1);
		};
		std::vector<dRealVector3> vertices;
		vertices.reserve((zData.size1()-1)*(zData.size2()-1)*3*2);
		
		std::vector<std::array<unsigned int,3>> faces;
		faces.reserve((zData.size1()-1)*(zData.size2()-1)*2);
		
		for(unsigned int indX=0; indX<zData.size1()-1; indX++){
			for(unsigned int indY=0; indY<zData.size1()-1; indY++){
				vertices.push_back(boost::assign::list_of<double>(xTicks[indX])(yTicks[indY])(zData(indX,indY)));
				vertices.push_back(boost::assign::list_of<double>(xTicks[indX+1])(yTicks[indY])(zData(indX+1,indY)));
				vertices.push_back(boost::assign::list_of<double>(xTicks[indX+1])(yTicks[indY+1])(zData(indX+1,indY+1)));
				std::array<unsigned int,3> tempArray={{static_cast<unsigned int>(vertices.size()-3),static_cast<unsigned int>(vertices.size()-2),static_cast<unsigned int>(vertices.size()-1)}};
				faces.push_back(tempArray);
				
				vertices.push_back(boost::assign::list_of<double>(xTicks[indX])(yTicks[indY])(zData(indX,indY)));
				vertices.push_back(boost::assign::list_of<double>(xTicks[indX])(yTicks[indY+1])(zData(indX,indY+1)));
				vertices.push_back(boost::assign::list_of<double>(xTicks[indX+1])(yTicks[indY+1])(zData(indX+1,indY+1)));
				tempArray={{static_cast<unsigned int>(vertices.size()-1),static_cast<unsigned int>(vertices.size()-2),static_cast<unsigned int>(vertices.size()-3)}};
				faces.push_back(tempArray);
			};
		};
		
		body->AddTriMesh( vertices, faces, offsetPosition, offsetRotation, color);
	};
	return;
};

void ProcessGeometricPrimitivesListNode(pugi::xml_node node, boost::shared_ptr<Bodies::BaseBody > body){
	unsigned int geomPrimCounter=0;
	for(auto it=node.begin(); it!=node.end(); it++){	
		if(std::string(it->name())=="GeometricPrimitive"){
			geomPrimCounter++;
			ProcessGeometricPrimitiveNode(*it, body, "GeometricPrimitiveNo"+(boost::lexical_cast<std::string>( geomPrimCounter )) );
		};
	}
	return;
}


void ProcessBodyNode(pugi::xml_node node, boost::shared_ptr< Universe > uni, std::string defaultName, bool isStatic=false){
	pugi::xml_node tempNode;
	
	std::string name=getValue<std::string>(node, "Name", defaultName);
	std::string material=getValue<std::string>(node, "Material", "StdMaterial");
		
	dRealVector3 initialPosition=boost::assign::list_of<dReal>(0)(0)(0);
	tempNode=node.child("InitialPosition").child("Vector3");
	if(tempNode){
		initialPosition=ProcessVector3Node(tempNode);
	};
	
	dRealMatrix3 initialRotation=boost::numeric::ublas::identity_matrix<dReal>(3);
	tempNode=node.child("InitialRotation").child("Matrix3");
	if(tempNode){
		initialRotation=ProcessMatrix3Node(tempNode);
	};	
	
	dReal mass=getValue<dReal>(node, "Mass", 1);
	dRealVector3 massOffset=boost::assign::list_of<dReal>(0)(0)(0);
	tempNode=node.child("MassOffset").child("Vector3");
	if(tempNode){
		massOffset=ProcessVector3Node(tempNode);
	};

	dRealMatrix3 inertiaTensor=boost::numeric::ublas::identity_matrix<dReal>(3);
	tempNode=node.child("InertiaTensor").child("Tensor3");
	if(tempNode){
		inertiaTensor=ProcessTensor3Node(tempNode);
	};
	for(unsigned int row=1;row<=2;row++){
		for(unsigned int col=0;col<row;col++){
			if(inertiaTensor(col,row)!=0 && inertiaTensor(row,col)!=inertiaTensor(col,row)){
				inertiaTensor(row,col)=inertiaTensor(col,row);
			};
		};
	}
	
	boost::shared_ptr<Bodies::BaseBody> newBody;
	if(isStatic){
		newBody=uni->AddStaticBody(	name,
						mass,
						inertiaTensor,
						initialPosition,
						initialRotation,
						massOffset,
						material);
	}else{
		newBody=uni->AddRigidBody(	name,
						mass,
						inertiaTensor,
						initialPosition,
						initialRotation,
						massOffset,
						material);
	}
	tempNode=node.child("GeometricPrimitivesList");
	if(tempNode){
		ProcessGeometricPrimitivesListNode(tempNode, newBody);
	};
}

void ProcessBioFlexRotatoryNode(pugi::xml_node node, boost::shared_ptr< Universe > uni){
	pugi::xml_node tempNode;
	
	unsigned char bioFlexBusId=getValue<unsigned char>(node, "BioFlexBusId");
	
	dRealVector3 anchor;
	tempNode=node.child("Anchor").child("Vector3");
	if(tempNode){
		anchor=ProcessVector3Node(tempNode);
	}else{
		std::cout<<"In order to create a BioFlexRotatory instance (with BioFlexBusId "<< int(bioFlexBusId) << "), an anchor must be specified! This process will be cancelled."<<std::endl;
		return;
	};
	

	dRealVector3 axis;
	tempNode=node.child("Axis").child("Vector3");
	if(tempNode){
		axis=ProcessVector3Node(tempNode);
	}else{
		std::cout<<"In order to create a BioFlexRotatory instance (with BioFlexBusId "<< int(bioFlexBusId) << "), an axis must be specified! This process will be cancelled."<<std::endl;
		return;
	};

	std::string bodyInputName=getValue<std::string>(node, "BodyInput");
	
	boost::shared_ptr<Bodies::BaseBody> bodyInput=uni->GetBody(bodyInputName);

	std::string bodyOutputName=getValue<std::string>(node, "BodyOutput");
	boost::shared_ptr<Bodies::BaseBody> bodyOutput=uni->GetBody(bodyOutputName);
	
	
	dReal initialOutputAngle=getValue<dReal>(node, "InitialOutputAngle", 0);
	double springConstant=getValue<double>(node, "SpringConstant", 1);
	double dampingConstant=getValue<double>(node, "DampingConstant", 0);	
	
	boost::shared_ptr<BioFlexRotatory>tempDrive= uni->AddBioFlexRotatory(bioFlexBusId,
										anchor,
										axis,
										bodyInput,
										bodyOutput,
										initialOutputAngle,
										springConstant,
										dampingConstant);
	
}

void ProcessImuNode(pugi::xml_node node, boost::shared_ptr< Universe > uni){
	pugi::xml_node tempNode;
	
	unsigned char bioFlexBusId=getValue<unsigned char>(node, "BioFlexBusId");
	
	std::string bodyName=getValue<std::string>(node, "Body");
	
	dRealVector3 offsetPosition=boost::assign::list_of<dReal>(0)(0)(0);
	tempNode=node.child("OffsetPosition").child("Vector3");
	if(tempNode){
		offsetPosition=ProcessVector3Node(tempNode);
	};
	
	dRealMatrix3 offsetRotation=boost::numeric::ublas::identity_matrix<dReal>(3);
	tempNode=node.child("OffsetRotation").child("Matrix3");
	if(tempNode){
		offsetRotation=ProcessMatrix3Node(tempNode);
	};
	
	boost::shared_ptr<Imu> tempImu= uni->AttachImuToBody(	bioFlexBusId, 
								bodyName,
								offsetPosition, 
								offsetRotation);
}

void ProcessPressureSensorNode(pugi::xml_node node, boost::shared_ptr< Universe > uni){
	pugi::xml_node tempNode;
	
	unsigned char bioFlexBusId=getValue<unsigned char>(node, "BioFlexBusId");
	
	std::string bodyName=getValue<std::string>(node, "Body");
	
	dRealVector3 offsetPosition=boost::assign::list_of<dReal>(0)(0)(0);
	tempNode=node.child("OffsetPosition").child("Vector3");
	if(tempNode){
		offsetPosition=ProcessVector3Node(tempNode);
	};
	
	dRealMatrix3 offsetRotation=boost::numeric::ublas::identity_matrix<dReal>(3);
	tempNode=node.child("OffsetRotation").child("Matrix3");
	if(tempNode){
		offsetRotation=ProcessMatrix3Node(tempNode);
	};
	
	boost::shared_ptr<PressureSensor> tempPressureSensor= uni->AttachPressureSensorToBody(	bioFlexBusId, 
												bodyName,
												offsetPosition, 
												offsetRotation);
}


boost::shared_ptr< Universe > GeometryXmlParser::Process(std::istream& stream, boost::shared_ptr< Universe > uni){
	pugi::xml_document doc;
	pugi::xml_parse_result result = doc.load(stream);
	if(!result){
		throw std::invalid_argument("Could not parse the stream.");
	}
	
	auto tempNode=doc.first_child().child("Universe");
	if(tempNode){
		uni->Clear(); // If a "Universe" node was defined in the xml, the universe should be recreated.
	};
	
	unsigned int bodyCounter=0;
	for(auto it=doc.first_child().begin(); it!=doc.first_child().end(); it++){
		if(std::string(it->name())=="RigidBody"){
			ProcessBodyNode(*it, uni, "BodyNo"+(boost::lexical_cast<std::string>( bodyCounter )), false);
			bodyCounter++;
		}else if(std::string(it->name())=="StaticBody"){
			ProcessBodyNode(*it, uni, "BodyNo"+(boost::lexical_cast<std::string>( bodyCounter )), true);
			bodyCounter++;
		};
	};
	
	for(auto it=doc.first_child().begin(); it!=doc.first_child().end(); it++){
		if(std::string(it->name())=="BioFlexRotatory"){
			ProcessBioFlexRotatoryNode(*it, uni);
		};
	};
	
	for(auto it=doc.first_child().begin(); it!=doc.first_child().end(); it++){
		if(std::string(it->name())=="Imu"){
			ProcessImuNode(*it, uni);
		};
	};
	
	for(auto it=doc.first_child().begin(); it!=doc.first_child().end(); it++){
		if(std::string(it->name())=="PressureSensor"){
			ProcessPressureSensorNode(*it, uni);
		};
	};
	
	
	return uni;
};

boost::shared_ptr< Universe > GeometryXmlParser::Process(std::string xml, boost::shared_ptr< Universe > uni){
	std::ifstream ifs;
	ifs.open(xml, std::ifstream::in);
	if(ifs.good()){
		return Process(ifs, uni);
	}else{
		std::stringstream tempStream(xml);
		return Process(tempStream, uni);
	}
};