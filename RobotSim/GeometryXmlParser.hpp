#ifndef GEOMETRY_XMLPARSER_HPP
#define GEOMETRY_XMLPARSER_HPP

// STL includes
#include <istream>

// Boost includes
#include <boost/shared_ptr.hpp>

// Own header files
#include "OdeDrawstuff.hpp"
#include "Universe.hpp"

namespace GeometryXmlParser{
	boost::shared_ptr< Universe > Process(std::string filename, boost::shared_ptr<Universe> uni);
	boost::shared_ptr< Universe > Process(std::istream& stream, boost::shared_ptr< Universe > uni);
};
#endif