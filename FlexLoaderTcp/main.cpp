#include <stdlib.h>
#include <functional>
#include <vector>

#include <boost/asio.hpp>
#include <boost/assign.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>
#include <boost/thread.hpp>


#include <boost/tokenizer.hpp>
#include <boost/token_functions.hpp> 

#include <BfbMessage.hpp>
#include "FlexLoader.hpp"

#include <boost/bind.hpp>
#include <boost/optional.hpp>
#include <boost/system/error_code.hpp>
#include <boost/thread/detail/thread.hpp>





////////////////////////////////////////// Main
int main (int argc, char **argv)
{	
	std::vector<std::string> tempBioFlexBusIds;	
	
	/** Declare and parse the program options specified via command line. */
	// Declare the supported options.
	boost::program_options::options_description desc("Command line options");
	desc.add_options()//This command continues over the next few lines.
	("help", "produce help message")
	("ip", boost::program_options::value<std::string>()->default_value("localhost"), "Defines the IP of the server.")
	("port", boost::program_options::value<std::string>()->default_value("50002"), "Defines the TCP port on which the server listens.")
	("hexfile", boost::program_options::value<std::string>()->default_value("flexdrive_firmware.hex"), "Defines the name of the firmware hex-file.")
	("bfbid", boost::program_options::value<std::vector<std::string>>(&tempBioFlexBusIds)->multitoken(), "Defines the BioFlexBus-ID of the client whose firmware should be updated.")
	("all", "If this option is set, the FlexLoader tries to update all of the 18 standard clients of the robot 'Hector'. These include the clients with the IDs 0x11, 0x12, 0x13, 0x21, 0x22, 0x23,..., 0x61, 0x62, 0x63.")
	;
	
	//Parse the options
	boost::program_options::variables_map vm;
	boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
	boost::program_options::notify(vm);    
	
	// Modify the program behaviour accordingly
	if (vm.count("help")) { // Show the help text and stop the program.
		std::cout << desc << std::endl;
		return 1;
	}

	std::vector<unsigned char> bioFlexBusIds;
	
	// Modify the program behaviour accordingly
	if (vm.count("all")) { // Show the help text and stop the program.
		for(unsigned int i=1; i<=6; i++){
			for(unsigned int j=1; j<=3; j++){
				bioFlexBusIds.push_back(i*16+j);
			};
		}
	}
	
	
	for(auto it=tempBioFlexBusIds.begin(); it!=tempBioFlexBusIds.end(); it++){
		unsigned int tempInt=std::stoul( *it , nullptr, 0 );
		if(tempInt<256){
			bioFlexBusIds.push_back(tempInt);
		};
	}
	
	
	std::string portNum=vm["port"].as<std::string>();
	std::string firmwareName=vm["hexfile"].as<std::string>();
	std::string ipAddress=vm["ip"].as<std::string>();
	
	boost::asio::io_service ioService;
	boost::asio::io_service::work work(ioService);
	boost::thread ioServiceThread(boost::bind(&boost::asio::io_service::run,&ioService)); // Start the IO-handler needed for communication.
	
	boost::asio::ip::tcp::resolver resolver(ioService);
	boost::asio::ip::tcp::resolver::query query(ipAddress, portNum);
	boost::asio::ip::tcp::resolver::iterator endpointIterator = resolver.resolve(query);
	boost::asio::ip::tcp::socket socket(ioService);
	boost::asio::connect(socket, endpointIterator);
	boost::asio::ip::tcp::resolver::iterator end;
	boost::system::error_code error = boost::asio::error::host_not_found;
	while (error && endpointIterator != end)
	{
		socket.close();
		socket.connect(*endpointIterator++, error);
	}
	if (error){
		throw boost::system::system_error(error);
	}
	
	

	std::list<unsigned char> successfulTransmits;
	std::list<unsigned char> unsuccessfulTransmits;
	for(auto it=bioFlexBusIds.begin(); it!=bioFlexBusIds.end(); it++){
		unsigned int successStatus=flexloader(&socket, *it, firmwareName);
		if(successStatus==0){
			successfulTransmits.push_back(*it);
		}else{
			unsuccessfulTransmits.push_back(*it);
		};
	}

	std::cout<<std::endl;
	for(auto it=0; it<=40; it++){
		std::cout<<"*";
	}
	std::cout<<std::endl;
	
	if(successfulTransmits.size()==bioFlexBusIds.size()){
		std::cout<<"The firmware has been successfully transmitted to all the specified clients."<<std::endl;
	}else if(unsuccessfulTransmits.size()==bioFlexBusIds.size()){
		std::cout<<"The firmware could not be transmitted to any of the specified clients."<<std::endl;
	}else{
		std::cout<<"The firmware could be successfully transmitted to some, but not to all of the specified clients."<<std::endl;
	}
	
	if(successfulTransmits.size()>0){
		std::cout<<"The new firmware runs now on the clients with the following IDs:"<<std::endl;
		for(auto it=successfulTransmits.begin(); it!=successfulTransmits.end(); it++){
			std::cout<<std::dec<<std::right<<std::setw(4)<<int(*it)<<"( "<<std::hex<< std::showbase<<int(*it)<<" ), "<<std::endl;;
		}
	}
	
	if(unsuccessfulTransmits.size()>0){
		std::cout<<"The new firmware DOES NOT run on the clients with the following IDs:"<<std::endl;
		for(auto it=unsuccessfulTransmits.begin(); it!=unsuccessfulTransmits.end(); it++){
			std::cout<<std::dec<<std::right<<std::setw(4)<<int(*it)<<"( "<<std::hex<< std::showbase<<int(*it)<<" ), "<<std::endl;;
		}
	}
	
	ioService.stop();
	ioServiceThread.join();
	
	return 0;
}
