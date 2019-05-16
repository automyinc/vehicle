/*
 * ublox_receiver.cpp
 *
 *  Created on: May 16, 2019
 *      Author: mad
 */

#include <automy/vehicle/UbloxReceiver.h>
#include <automy/vehicle/GPS_Info.hxx>

#include <vnx/Config.h>
#include <vnx/Process.h>
#include <vnx/Thread.h>

using namespace automy;


class ProcessThread : public vnx::Thread {
public:
	ProcessThread(const std::string& vnx_name_) : Thread(vnx_name_) {}
	
	void main() {
		subscribe("vehicle.gps_info");
		
		while(vnx_do_run) {
			/*
			 * Wait for next Message to be received.
			 */
			std::shared_ptr<const vnx::Message> msg = read_blocking();
			if(!msg) {
				break;		// shutdown has been triggered
			}
			
			/*
			 * See if we got a GPS_Info and print it out.
			 */
			std::shared_ptr<const vnx::Sample> sample = std::dynamic_pointer_cast<const vnx::Sample>(msg);
			if(sample) {
				auto value = std::dynamic_pointer_cast<const vehicle::GPS_Info>(sample->value);
				if(value) {
					std::cout << (*value) << std::endl;
				}
			}
		}
	}
	
};


int main(int argc, char** argv) {
	
	std::map<std::string, std::string> options;
	options["p"] = "port";
	options["port"] = "device path";
	
	vnx::init("ublox_receiver", argc, argv, options);
	
	{
		vnx::Handle<vehicle::UbloxReceiver> module = new vehicle::UbloxReceiver("UbloxReceiver");
		module->output = "vehicle.gps_info";
		vnx::read_config("port", module->port);		// default = "/dev/ttyACM0"
		module.start_detached();
	}
	
	/*
	 * Start our processing thread.
	 */
	ProcessThread thread("ProcessThread");
	thread.start();
	
	/*
	 * Wait for shutdown.
	 */
	vnx::wait();
}



