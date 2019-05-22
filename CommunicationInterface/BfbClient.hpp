#ifndef BFBCLIENT_HPP
#define BFBCLIENT_HPP

// STL includes
#include <functional>
#include <list>
#include <map>
#include <string>
#include <vector>

// Boost includes
#include <boost/assign.hpp>
#include <boost/shared_ptr.hpp>

// Own header files
#include "Attribute.hpp"
#include <BfbMessage.hpp>

class BfbClient{
	private:
		#ifndef SWIG
		unsigned char BioFlexBusId;
		bool FatalError;
		std::map<std::string, boost::shared_ptr<Attribute>> Attributes;//=std::map<std::string, boost::shared_ptr<Attribute>>();
		AttributeUtilities::sendMessageHandle_t SendMessageHandle;
		std::function<unsigned long int()> GetIterationNumberHandle;
		void NotifyOfError();
		#endif
	public:
		BfbClient(unsigned char bioFlexBusId, AttributeUtilities::sendMessageHandle_t sendMessageHandle, std::function<unsigned long int()> getIterationNumberHandle, std::vector<Attribute> attributes=std::vector<Attribute>());
		AttributeUtilities::variableTypeList GetValue(std::string attributeName);
		unsigned char GetBioFlexBusId();
		void UpdateValue(std::string attributeName);
		void UpdateValueIfTooOld(std::string attributeName);
		void UpdateValueIfTooOldAndWait(std::string attributeName);
		void SetValue(std::string attributeName, AttributeUtilities::variableTypeList value);
		void SetValue(std::string attributeName, AttributeUtilities::variableTypeList value, bool blocking);
		//void SetValue(std::string attributeName, double value, bool blocking=false);
		//void SetValue(std::string attributeName, signed long int value, bool blocking=false);
		bool GetErrorState();
		void ClearErrorState();
		void AddAttribute(const Attribute attribute);
		void AddAttribute(std::string name, unsigned char protocolId, unsigned char commandId, const std::vector<AttributeUtilities::Entry> entries, const bool isTransmittable=false, const bool confirmTransmit=false);
		
};
#endif
