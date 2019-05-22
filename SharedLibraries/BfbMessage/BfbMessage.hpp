#ifndef BFBMESSAGE_H
#define BFBMESSAGE_H

// STL includes
#include <stdexcept>
#include <type_traits>
#include <vector>

// Boost includes
#include <boost/assign.hpp>
#include <boost/date_time.hpp>

class BfbMessage;

namespace BfbConstants{
	enum fieldPosition_t{
		destinationPos		=   0,
		sourcePos		=   1,
		flagsPos		=   2
	};
		
	enum shortMessageFieldPosition_t{
		shortProtocolPos	=   3, 
		shortCommandPos		=   4, 
		shortPayloadStart	=   5,
		shortCrcPos		=   7
	};
	const unsigned int shortLength	=   8;

	enum longMessageFieldPosition_t{
		longLengthPos		=   3,
		longHeaderCrcPos	=   4,
		longProtocolPos		=   5, 
		longCommandPos		=   6,
		longPayloadStart	=   7,
		longPayloadCrcStart	=  -2
	};
	const unsigned char longMessageOverhead=  9;
	const unsigned char longMessageMaxLength=152;
	
	enum ultraLongMessageFieldPosition_t{
		ultraLongLengthStart	=   3,
		ultraLongHeaderCrcPos	=   7,
		ultraLongProtocolPos	=   8, 
		ultraLongCommandPos	=   9,
		ultraLongPayloadStart	=   10,
		ultraLongPayloadCrcStart=  -2
	};
	const unsigned char ultraLongMessageOverhead=  12;
	const unsigned long ultraLongMessageMaxLength= pow(2,32);
	

	enum dummyCrc_t{
		shortCrcDummy		 =0xAA,
		longHeaderCrcDummy	 =0xDB,
		longPayloadCrcDummy1	 =0x66,
		longPayloadCrcDummy2	 =0x00,
		ultraLongHeaderCrcDummy	 =0xDB+0x01,
		ultraLongPayloadCrcDummy1=0x66+0x01,
		ultraLongPayloadCrcDummy2=0x00+0x01
	};

	enum flagFieldBitMask_t{
		sizeFlag_bm		=0x18,
		ultraLongPacketFlag_bm	=0x08,
		longPacketFlag_bm	=0x10,
		errorFlag		=0x40,
		busAllocationFlag_bm	=0x80
	};
}


namespace BfbFunctions{
	/*!\brief The function tests a vector whether it contains the elements of a valid short packet. 
	 * That means that the length flag must be correctly set (therefore, it should be 0), the length of the vector must be 8 and the (dummy) CRC must be correct.
	 * \param raw_data A vector of unsigned char that holds the raw data of the potentially valid message.
	 * \return A boolean that represents the result of the test.
	 */
	bool isValidShortPacket(std::vector<unsigned char> rawData);
	
	/*!\brief The function tests a vector whether it contains the elements of a valid long packet header. 
	 * That means that the length flag must be correctly set (therefore, it should be 1), the length of the vector must be smaller than or equal the specified length (length byte plus extended length bits) and the (dummy) header CRC must be correct.
	 * \param raw_data A vector of unsigned char that holds the raw data of the potentially valid message header.
	 * \return A boolean that represents the result of the test.
	 */
	bool isValidLongPacketHeader(std::vector<unsigned char> rawData);

	bool isValidUltraLongPacketHeader(std::vector<unsigned char> rawData);
	
	/*!\brief The function tests a vector whether it contains the elements of a valid long packet. 
	 * That means that the vector must pass all the tests of the isValidLongPacketHeader function, the length of the vector must equal the specified length (length byte plus extended length bits) and the (dummy) payload CRCs must be correct.
	 * \param raw_data A vector of unsigned char that holds the raw data of the potentially valid message.
	 * \return A boolean that represents the result of the test.
	 */
	bool isValidLongPacket(std::vector<unsigned char> rawData);
	
	bool isValidUltraLongPacket(std::vector<unsigned char> rawData);
	
	/*!\brief The function returns the number of bytes that are missing in order to complete a message. This may be either a short or a long message. 
	 * Typically, a vector with a length of 8 elements will be supplied. 
	 * If the supplied bytes are insufficient to decide whether the vector contains parts of a valid message (e.g. if the vector is too small), 
	 * the function returns the number of bytes that are needed for this decision. Therefore, even after the specified number of bytes has been 
	 * appended to the input vector, the message might not be complete.
	 * \param raw_data A vector of unsigned char that holds the raw data of the (partial) message.
	 * \return The number of bytes that are missing to build a potentially valid message or at least to return a valid result. 
	 */
	unsigned long numOfMissingBytes(std::vector<unsigned char> rawData);
	
	void printMessage(const BfbMessage &message, std::string foreword="", signed long int maxPayloadPrintout=-1);
	void printMessage(boost::shared_ptr<const BfbMessage> message, std::string foreword="", signed long int maxPayloadPrintout=-1);
	
	
	double clip(double input, double lower, double upper);
	
	std::vector<unsigned char> convertDoubleToBytes(const double input, const unsigned char numOfBits, const bool isSigned);
	std::vector<unsigned char> convertDoublesToBytes(std::vector<double> input, unsigned char numOfBits, bool isSigned);
	
	template <class T>
	std::vector<unsigned char> convertIntegralsToBytes(std::vector<T> input){
		if(!std::is_integral<T>::value){
			throw std::invalid_argument("The passed type is not a valid integral type.");
		};
		if(input.size()==0){
			return std::vector<unsigned char>();
		};
		unsigned char numOfBytes=sizeof(input.at(0))/8;
		const bool isSigned=std::is_signed<T>::value;
		
		std::vector<unsigned char> output;
		output.reserve(input.size()*numOfBytes);
		for(auto it=input.begin(); it!=input.end(); it++){
			auto temp=convertDoubleToBytes(*it, numOfBytes, isSigned);
			output.insert(output.end(), temp.begin(), temp.end());
		};
		return output;
	}
	
	template <class T>
	std::vector<unsigned char> convertIntegralToBytes(T input){
		return convertIntegralsToBytes(boost::assign::list_of<T>(input));
	};
	
}

class BfbMessage{
	private:
		unsigned char Destination	=	0;
		unsigned char Source		=	0;
		bool  	BusAllocation		=   false;
		bool	Error			=   false;
		unsigned char Protocol		=	0;
		unsigned char Command		=	0;
		std::vector<unsigned char> Payload;
		std::string Comment		=      "";
		
	public:
		//unsigned char MaximumNumberOfTransmissions	=5;
		//unsigned char NumberOfTransmissions		=0;
		//boost::posix_time::ptime TimeOfLastTransmission;
		/*!\brief Class constructor for BfbMessage.
		 *
		 *	The instance will be initialised with all attributes set to zero.
		 *
		 */
		BfbMessage();
		
		/*!\brief Class constructor for BfbMessage.
		 *
		 *  The instance will be initialised based on the provided raw data.
		 *  If the provided data does not fit to the specifications of the BioFlexBus protocol, an invalid_argument exception will be thrown. 
		 * \param raw_data A pointer to an array that holds the data.
		 * \param raw_data_length Length of the aforementioned array.
		 *  
		 */
		BfbMessage(unsigned char* rawData, uint16_t rawDataLength);
		
		/*!\brief Class constructor for BfbMessage.
		 *
		 *  The instance will be initialised based on the provided raw data.
		 *  If the provided data does not fit to the specifications of the BioFlexBus protocol, an invalid_argument exception will be thrown. 
		 * \param raw_data A vector of unsigned char that holds the raw data of the message.
		 *  
		 */
		BfbMessage(std::vector<unsigned char> rawData);
		
		
		BfbMessage(unsigned char destination, unsigned char source, bool busAllocation, bool Error, unsigned char protocol, unsigned char command, std::vector<unsigned char> payload=std::vector<unsigned char>(), std::string comment="");
		
		BfbMessage(const BfbMessage &message);
		
		virtual ~BfbMessage();
		/*!\brief Get the destination address byte of the message.
		 *
		 * \return A unsigned char specifying the destination.
		 */
		unsigned char GetDestination() const;
		
		/*!\brief Get the source address byte of the message.
		 *
		 * \return A unsigned char specifying the source.
		 */
		unsigned char GetSource() const;
		
		/*!\brief Get the protocol id of the message.
		 *
		 * \return A unsigned char specifying the protocol.
		 */
		unsigned char GetProtocol() const;
		
		/*!\brief Get the command id of the message.
		 *
		 * \return A unsigned char specifying the command.
		 */
		unsigned char GetCommand() const;
		
		/*!\brief Get the bus allocation flag of the message.
		 *
		 * \return (bool) true if bus allocation flag is set, false if not.
		 */
		bool GetBusAllocation() const;
		
		/*!\brief Get the error flagdata[BFB_LONG_LENGTH_POS] of the message.
		 *
		 * \return (bool) true if error flag is set, false if not.
		 */
		bool GetError() const;
		
		/*!\brief Get the payload of the message.
		 *
		 * \return A vector of unsigned char holding the payload.
		 */
		std::vector<unsigned char> GetPayload() const;
		
		/*!\brief Get the raw data of the message as it can be send via TCP or another bus.
		 *
		 * \return A vector of unsigned char holding the raw data .
		 */
		std::vector<unsigned char> GetRawData() const;
		
		std::string GetComment() const;
		
		/*!\brief Set the destination address byte of the message.
		 * \param new_destination An unsigned char value specifying the address of the recipient.
		 */
		void SetDestination(unsigned char newDestination);
		
		/*!\brief Set the source address byte of the message.
		 * \param new_source An unsigned char value specifying the address of the sender.
		 */
		void SetSource(unsigned char newSource);
		
		/*!\brief Set the protocol id of the message.
		 * \param new_payload An unsigned char value specifying the new protocol id.
		 */
		void SetProtocol(unsigned char newProtocol);
		
		/*!\brief Get the command id of the message.
		 * \param new_command An unsigned char value specifying the new command id.
		 */
		void SetCommand(unsigned char newCommand);

		/*!\brief Set the bus allocation flag of the message.
		 * \param new_bus_allocation_flag A boolean flag specifying whether the message's sender assigns the bus to it's recipient.
		 */
		void SetBusAllocationFlag(bool newBusAllocation);
		
		/*!\brief Set the error flag of the message.
		 * \param new_error_flag A boolean flag specifying whether the message's sender has detected any sort of error.
		 */
		void SetErrorFlag(bool newError);

		/*!\brief Set the payload of the message.
		 * \param new_payload A vector of unsigned char values specifying the new payload.
		 */
		void SetPayload(std::vector<unsigned char>  newPayload);

		/*!\brief Set the raw data of the message as it can be send via TCP (or another bus).
		 * \param raw_data A pointer to an array that holds the data.
		 * \param raw_data_length Length of the aforementioned array.
		 * \return A boolean feedback whether the data format etc. was correct (true means correct; false means incorrect).
		 */
		void SetRawData(unsigned char* rawData, uint16_t rawDataLength);

		/*!\brief Set the raw data of the message as it can be send via TCP (or another bus).
		 * \param new_raw_data A vector holding the raw data of a message.
		 * \return A boolean feedback whether the data format etc. was correct (true means correct; false means incorrect).
		 */
		void SetRawData(std::vector<unsigned char>  newRawData);		
		
		void SetComment(std::string comment);
};


#endif
