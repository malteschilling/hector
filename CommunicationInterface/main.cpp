// STL includes
#include <fstream>
#include <iostream>
#include <mutex>
#include <queue>
#include <stdlib.h>
#include <string.h>
#include <typeinfo>

// Boost includes
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/assign.hpp>
#include <boost/assign/ptr_list_of.hpp>
#include <boost/bind.hpp>
#include <boost/date_time.hpp>
#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/program_options.hpp>

// Own header files
#include "Attribute.hpp"
#include "BfbClient.hpp"
#include <BfbMessage.hpp>
#include "CommunicationInterface.hpp"
#include "CommunicationXmlParser.hpp"

////////////////////////////////////////// Main
int main (int argc, char **argv)
{	
	auto tempComm=boost::make_shared<CommunicationInterface>("localhost", "50002");
	tempComm->ParseProtocolXml("../BioFlexBusProtocolXmls/BIOFLEX_ROTATORY_1_PROT.xml");
	tempComm->ParseProtocolXml("../BioFlexBusProtocolXmls/BIOFLEX_ROTATORY_CONTROL_1_PROT.xml");
	tempComm->ParseProtocolXml("../BioFlexBusProtocolXmls/SIMSERV_1_PROT.xml");
	tempComm->ParseProtocolXml("../BioFlexBusProtocolXmls/IMU_PROT.xml");
	tempComm->ParseProtocolXml("../BioFlexBusProtocolXmls/PRESSURE_SENSOR_PROT.xml");
	auto tempClient0=tempComm->CreateBfbClient(18+16*0, boost::assign::list_of<std::string>("BIOFLEX_ROTATORY_1_PROT")("BIOFLEX_ROTATORY_CONTROL_1_PROT"));
	auto tempClient1=tempComm->CreateBfbClient(18+16*1, boost::assign::list_of<std::string>("BIOFLEX_ROTATORY_1_PROT")("BIOFLEX_ROTATORY_CONTROL_1_PROT"));
	auto tempClient2=tempComm->CreateBfbClient(18+16*2, boost::assign::list_of<std::string>("BIOFLEX_ROTATORY_1_PROT")("BIOFLEX_ROTATORY_CONTROL_1_PROT"));
	auto tempTimer=tempComm->CreateBfbClient(14, boost::assign::list_of<std::string>("SIMSERV_1_PROT"));
	
	AttributeUtilities::variableTypeList tempList;
	std::ifstream ifs;
	ifs.open("Hector.xml", std::ifstream::in);
	auto text=std::string(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
	tempList.clear();
	tempList.push_back(text);
	tempTimer->SetValue("geometryXml", tempList);

	
	
	for(unsigned int i=0;i<500;i++){
		AttributeUtilities::variableTypeList tempVarTypList=tempClient0->GetValue("outputPosition");
		std::cout<<tempVarTypList[0]<<std::endl;
		tempVarTypList=tempClient1->GetValue("outputPosition");
		std::cout<<tempVarTypList[0]<<std::endl;
		tempVarTypList=tempClient2->GetValue("outputPosition");
		
		//std::cout<<"Current output position: "<<tempVarTypList[0]<<std::endl;
		tempList.clear();
		tempList.push_back(1);
		tempClient0->SetValue("desiredValue_ISC", tempList);
		tempClient1->SetValue("desiredValue_ISC", tempList);
		tempClient2->SetValue("desiredValue_ISC", tempList);
		tempComm->NotifyOfNextIteration();
		
		tempList.clear();
		tempList.push_back(0.01);
		tempTimer->SetValue("relTimerMs", tempList);
	};
	//std::ifstream is ("./test.xml");
	/*  while (is.good())          // loop while extraction from file is possible
		{
		char c = is.get();       // get character from file
		if (is.good())
		std::cout << c;
		}
		std::cout<<std::endl;
		sleep(1);
	*/
	//XmlParser::Process(is);
	return 0;
}
