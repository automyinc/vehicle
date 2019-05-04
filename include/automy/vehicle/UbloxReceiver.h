/*
 * UbloxReceiver.h
 *
 *  Created on: Dec 11, 2017
 *      Author: mad
 */

#ifndef SENSORS_INCLUDE_SENSORS_UBLOXRECEIVER_H_
#define SENSORS_INCLUDE_SENSORS_UBLOXRECEIVER_H_

#include <automy/vehicle/UbloxReceiverBase.hxx>
#include <automy/vehicle/GPS_Info.hxx>


namespace ublox {
	struct NavSol;
	struct NavPosLLH;
	struct NavVelNed;
	struct NavGPSTime;
} // ublox


namespace automy {
namespace vehicle {

class UbloxReceiver : public UbloxReceiverBase {
public:
	UbloxReceiver(const std::string& _vnx_name);
	
protected:
	void main() override;
	
	int open_port();
	
	void handle(std::shared_ptr<const vehicle::UBX_Packet> value);
	
	void handle(std::shared_ptr<const vehicle::PPS_Signal> value);
	
	void handle(const ublox::NavSol& nav_sol, int64_t time);
	
	void handle(const ublox::NavPosLLH& nav_pos_llh, int64_t time);
	
	void handle(const ublox::NavVelNed& msg, int64_t time);
	
	void handle(const ublox::NavGPSTime& msg, int64_t time);
	
	void check_packets();
	
	void read_loop(const int fd, std::shared_ptr<vnx::Topic> topic);
	
private:
	vehicle::GPS_Info gps_info;
	
	uint32_t nav_sol_tow = 0;
	uint32_t nav_pos_tow = 0;
	uint32_t nav_vel_tow = 0;
	uint32_t nav_time_tow = 0;
	int64_t last_pps_time = 0;
	
};


} // vehicle
} // automy

#endif /* SENSORS_INCLUDE_SENSORS_UBLOXRECEIVER_H_ */
