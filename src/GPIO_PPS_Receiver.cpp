/*
 * GPIO_PPS_Receiver.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: mad
 */

#include <automy/vehicle/GPIO_PPS_Receiver.h>
#include <automy/vehicle/PPS_Signal.hxx>

#include <vnx/Process.h>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>


namespace automy {
namespace vehicle {

GPIO_PPS_Receiver::GPIO_PPS_Receiver(const std::string& _vnx_name) : GPIO_PPS_ReceiverBase(_vnx_name) {}

void GPIO_PPS_Receiver::main() {
	
	const int fd = ::open(device.c_str(), O_RDWR);
	if(fd < 0) {
		log(ERROR).out << "open('" << device << "') failed!";
		return;
	}
	
	int64_t last_time = 0;
	while(vnx::do_run()) {
		::lseek(fd, 0, SEEK_SET);
		
		char dummy = 0;
		const int res = ::read(fd, &dummy, 1);		// clean-up interrupt flag
		
		::pollfd fdset[1] = {};
		fdset[0].fd = fd;
		fdset[0].events = POLLPRI;
		fdset[0].revents = 0;
		
		if(::poll(fdset, 1, 500) < 0) {
			log(ERROR).out << "poll() failed!";
			break;
		}
		
		if(fdset[0].revents & POLLPRI) {
			PPS_Signal out;
			out.time = vnx::get_time_micros();
			if(last_time) {
				out.jitter = std::abs(out.time - last_time - 1000000);
			}
			publish(out, output);
			
			log(INFO).out << "PPS " << out.time << ", jitter=" << out.jitter << " usec";
			last_time = out.time;
		}
		
	}
	
	::close(fd);
}


} // vehicle
} // automy
