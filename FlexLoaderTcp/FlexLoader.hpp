#ifndef FLEXLOADER_HPP
#define FLEXLOADER_HPP

#include <boost/asio.hpp>

int flexloader(boost::asio::ip::tcp::socket* socket, unsigned char id, const std::string hexfile);

#endif /* FLEXLOADER_HPP_INCLUDED */