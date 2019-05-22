#ifndef BFBCLIENT_HPP
#define BFBCLIENT_HPP
#include <array>
#include "BfbMessage.hpp"

class BfbClient{
	protected:
		unsigned char BioFlexBusId;
		unsigned char FatalError=false;
		std::array<unsigned char, 16> UniqueIdentificationData{{}}; // The BioFlex Protocol specifies that this information should be sent as reply for a message with protocol id 1 and command id 0. Currently, the data is created randomly. 
	public:
		BfbClient(unsigned char bioFlexBusId);
		virtual ~BfbClient(){};
		virtual boost::shared_ptr<BfbMessage> ProcessMessage(boost::shared_ptr<const BfbMessage> Message);
		//virtual void PreSimulationStepUpdate(); 
		virtual void PostSimulationStepUpdate(double deltaT); 
};
#endif