
// AUTO GENERATED by vnxcppcodegen

#ifndef INCLUDE_automy_vehicle_UbloxReceiver_CLIENT_HXX_
#define INCLUDE_automy_vehicle_UbloxReceiver_CLIENT_HXX_

#include <vnx/Client.h>
#include <automy/math/Vector3d.h>
#include <automy/vehicle/PPS_Signal.hxx>
#include <automy/vehicle/UBX_Packet.hxx>
#include <vnx/Module.h>
#include <vnx/TopicPtr.h>


namespace automy {
namespace vehicle {

class UbloxReceiverClient : public vnx::Client {
public:
	UbloxReceiverClient(const std::string& service_name);
	
	UbloxReceiverClient(vnx::Hash64 service_addr);
	
	void handle(const ::std::shared_ptr<const ::automy::vehicle::PPS_Signal>& sample);
	
	void handle_async(const ::std::shared_ptr<const ::automy::vehicle::PPS_Signal>& sample);
	
	void handle(const ::std::shared_ptr<const ::automy::vehicle::UBX_Packet>& sample);
	
	void handle_async(const ::std::shared_ptr<const ::automy::vehicle::UBX_Packet>& sample);
	
};


} // namespace automy
} // namespace vehicle

#endif // INCLUDE_automy_vehicle_UbloxReceiver_CLIENT_HXX_
