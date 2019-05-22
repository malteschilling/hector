/**
 * \file BfbProtocolIds.hpp The file contains a list of the protocol Ids that are used in the BioFlex protocol.
 * Note: A file with this name exists both in the server and the simulation. If a protocol ID changes, this change must be carried out in both files!!!
 */
#ifndef BFBPROTOCOLIDS_HPP
#define BFBPROTOCOLIDS_HPP
namespace BfbProtocolIds{
		enum ProtocolId{
			BIOFLEX_1_PROT=1,
			BIOFLEX_ROTATORY_1_PROT=13,
			BIOFLEX_ROTATORY_ERROR_PROT=15,
			BIOFLEX_ROTATORY_CONTROL1_PROT=16,
			SIMULATOR_OBJECT_PROT=1,
			SIMSERV_1_PROT=12,
			PRESSURE_SENSOR_PROT=17,
			IMU_SENSOR_PROT=18
		};
};
#endif