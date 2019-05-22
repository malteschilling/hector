#ifndef BFBMESSAGEPROCESSOR_HPP
#define BFBMESSAGEPROCESSOR_HPP

// Boost includes
#include <boost/shared_ptr.hpp>

// Own header files
#include "Universe.hpp"
#include "BfbMessage.hpp"


namespace BfbMessageProcessor{
	boost::shared_ptr<BfbMessage> ProcessMessage(boost::shared_ptr<Universe> universe, boost::shared_ptr<const BfbMessage> message);
};

#endif