/*
 * UbloxReceiver.cpp
 *
 *  Created on: Dec 11, 2017
 *      Author: mad
 */

#include <automy/vehicle/UbloxReceiver.h>

#include <vnx/Publisher.h>

#include <ublox/ublox_structures.h>

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>


namespace automy {
namespace vehicle {

UbloxReceiver::UbloxReceiver(const std::string& _vnx_name)
	:	UbloxReceiverBase(_vnx_name)
{
}

void UbloxReceiver::main() {
	subscribe(topic_msgs);
	subscribe(input_pps);
	
	int fd = -1;
	std::thread read_thread;
	if(port.size()) {
		fd = open_port();
		read_thread = std::thread(&UbloxReceiver::read_loop, this, fd, topic_msgs);
		log(INFO).out << "Running on " << port;
	}
	
	gps_info.antenna_pos = antenna_pos;
	
	try {
		Super::main();
	} catch(std::exception& ex) {
		log(ERROR).out << ex.what();
	}
	
	if(fd >= 0) {
		::close(fd);
	}
	if(read_thread.joinable()) {
		read_thread.join();
	}
}

int UbloxReceiver::open_port() {
	const int fd = ::open(port.c_str(), O_RDONLY | O_NOCTTY);
	if(fd < 0) {
		throw std::runtime_error("Cannot open device: " + port);
	}
	
	::termios options = {};
	::tcgetattr(fd, &options);
	
	int speed = B115200;
	switch(baudrate) {
		case 9600: speed = B9600; break;
		case 19200: speed = B19200; break;
		case 38400: speed = B38400; break;
		case 57600: speed = B57600; break;
		case 115200: speed = B115200; break;
		case 230400: speed = B230400; break;
		case 460800: speed = B460800; break;
		case 921600: speed = B921600; break;
		default: log(WARN).out << "Unsupported baudrate: " << baudrate;
	}
	::cfsetispeed(&options, speed);
	::cfsetospeed(&options, speed);
	
	::cfmakeraw(&options);						// magic
	options.c_cflag &= ~CSTOPB;					// magic
	options.c_cflag &= ~CRTSCTS;				// magic
	options.c_cflag |= CLOCAL | CREAD;			// only the gods know
	options.c_cc[VMIN] = 1;						// read at least one byte
	options.c_cc[VTIME] = 10;					// magic
	::tcsetattr(fd, TCSANOW, &options);
	return fd;
}

void UbloxReceiver::handle(std::shared_ptr<const vehicle::UBX_Packet> value) {
	if(value->class_id == 0x01) {
		switch(value->msg_id) {
			case 0x02: handle(*((const ublox::NavPosLLH*)value->payload.data()), value->time); break;
			case 0x06: handle(*((const ublox::NavSol*)value->payload.data()), value->time); break;
			case 0x20: handle(*((const ublox::NavGPSTime*)value->payload.data()), value->time); break;
			case 0x12: handle(*((const ublox::NavVelNed*)value->payload.data()), value->time); break;
		}
	}
}

void UbloxReceiver::handle(std::shared_ptr<const vehicle::PPS_Signal> value) {
	last_pps_time = value->time;
}

void UbloxReceiver::handle(const ublox::NavSol& msg, int64_t time) {
	nav_sol_tow = msg.iTOW;
	gps_info.have_fix = msg.gpsFix == 3 || msg.gpsFix == 4;		// 3D fix or optionally with dead-reckoning
	gps_info.num_sats = msg.numSV;
	check_packets();
	if(!gps_info.have_fix) {
		log(WARN).out << "Have no fix, num_sats=" << gps_info.num_sats;
	}
}

void UbloxReceiver::handle(const ublox::NavPosLLH& msg, int64_t time) {
	nav_pos_tow = msg.iTOW;
	gps_info.latitude = msg.latitude_scaled / 1e7;
	gps_info.longitude = msg.longitude_scaled / 1e7;
	gps_info.height = msg.height / 1e3;
	gps_info.hdop = msg.horizontal_accuracy / 1e3;
	gps_info.vdop = msg.vertical_accuracy / 1e3;
	check_packets();
}

void UbloxReceiver::handle(const ublox::NavVelNed& msg, int64_t time) {
	nav_vel_tow = msg.iTOW;
	gps_info.heading = msg.heading_scaled / 1e5;
	gps_info.speed = msg.speed / 1e2;
	gps_info.vel_north = msg.velocity_north / 1e2;
	gps_info.vel_east = msg.velocity_east / 1e2;
	gps_info.vel_up = -1 * msg.velocity_down / 1e2;
	check_packets();
}

void UbloxReceiver::handle(const ublox::NavGPSTime& msg, int64_t time) {
	nav_time_tow = msg.iTOW;
	gps_info.gps_time = (((int64_t(315964800) + int64_t(msg.week) * 604800) * 1000)
							+ int64_t(msg.iTOW)) * 1000 + msg.ftow / 1000;
	check_packets();
}

void UbloxReceiver::check_packets() {
	if(nav_sol_tow == nav_pos_tow && nav_pos_tow == nav_vel_tow && nav_vel_tow == nav_time_tow) {
		gps_info.time = vnx::get_time_micros();
		gps_info.time_of_week = nav_pos_tow;
		gps_info.have_pps = last_pps_time > 0;
		const int64_t diff = gps_info.time - gps_info.gps_time;
		if(gps_info.have_pps) {
			const int64_t offset = int64_t(nav_pos_tow % 1000) * 1000;
			const int64_t delay = gps_info.time - last_pps_time;
			if(delay >= 1000000) {
				log(WARN).out << "Delay too high: " << delay/1000 << " ms";
			} else if(delay < 10000) {
				log(WARN).out << "Delay too small: " << delay/1000 << " ms";
			} else {
				gps_info.time = last_pps_time + offset;
				if(offset == 0) {
					log(INFO).out << "Have PPS, delay=" << delay/1000 << " ms, diff=" << diff/1000 << " ms";
				}
			}
		} else {
			if(input_pps) {
				log(WARN).out << "Have no PPS, diff=" << diff/1000 << " ms";
			}
		}
		publish(gps_info, output);
	}
}

static void ubx_checksum(uint8_t byte, uint8_t crc[2]) {
	crc[0] = crc[0] + byte;
	crc[1] = crc[1] + crc[0];
}

void UbloxReceiver::read_loop(const int fd, std::shared_ptr<vnx::Topic> topic)
{
	enum {
		SYNC_1, SYNC_2,
		CLASS_ID, MSG_ID,
		LENGTH_1, LENGTH_2,
		PAYLOAD,
		CHECKSUM_1, CHECKSUM_2
	} state = SYNC_1;
	
	vnx::Publisher publisher;
	std::shared_ptr<UBX_Packet> packet;
	uint32_t length = 0;
	uint32_t index = 0;
	uint8_t crc[2] = {};
	uint8_t recv_crc[2] = {};
	bool have_sync = false;
	
	while(true) {
		uint8_t buf[1024];
		const int num_bytes = ::read(fd, buf, sizeof(buf));
		if(num_bytes <= 0) {
			break;
		}
		
		for(int i = 0; i < num_bytes; ++i) {
			const uint8_t c = buf[i];
			switch(state) {
			case SYNC_1:
				if(c == 0xB5) {
					state = SYNC_2;
				}
				break;
			case SYNC_2:
				if(c == 0x62) {
					state = CLASS_ID;
				} else {
					state = SYNC_1;
				}
				break;
			case CLASS_ID:
				packet = UBX_Packet::create();
				packet->time = vnx::get_time_micros();
				packet->class_id = c;
				state = MSG_ID;
				crc[0] = 0; crc[1] = 0;
				ubx_checksum(c, crc);
				break;
			case MSG_ID:
				packet->msg_id = c;
				state = LENGTH_1;
				ubx_checksum(c, crc);
				break;
			case LENGTH_1:
				length = c;
				state = LENGTH_2;
				ubx_checksum(c, crc);
				break;
			case LENGTH_2:
				length |= uint32_t(c) << 8;
				index = 0;
				if(!have_sync && length > 100) {
					vnx::log_warn().out << "UbloxReceiver: Packet too big: " << length << " bytes!";
					state = SYNC_1;
				} else if(length > 0) {
					packet->payload.resize(length);
					state = PAYLOAD;
				} else {
					state = CHECKSUM_1;
				}
				ubx_checksum(c, crc);
				break;
			case PAYLOAD:
				packet->payload[index++] = c;
				if(index >= length) {
					state = CHECKSUM_1;
				}
				ubx_checksum(c, crc);
				break;
			case CHECKSUM_1:
				recv_crc[0] = c;
				state = CHECKSUM_2;
				break;
			case CHECKSUM_2:
				recv_crc[1] = c;
				if(recv_crc[0] == crc[0] && recv_crc[1] == crc[1]) {
					publisher.publish(packet, topic);
					have_sync = true;
				} else {
					have_sync = false;
					vnx::log_warn().out << "UbloxReceiver: Checksum failed for " << length << " byte packet!";
				}
				state = SYNC_1;
			}
		}
	}
}


} // vehicle
} // automy
