#ifndef COMMUNICATIONXMLPARSER_HPP
#define COMMUNICATIONXMLPARSER_HPP

// Boost includes
#include <boost/shared_ptr.hpp>

// Own header files
#include "Attribute.hpp"


namespace CommunicationXmlParser{
	typedef std::list<Attribute> AttributeList;
	typedef std::map<std::string, AttributeList> AttributeMap;
	AttributeMap Process(std::string xml);
	AttributeMap Process(std::istream& stream);
};
#endif