
// AUTO GENERATED by vnxcppcodegen

#include <automy/vehicle/package.hxx>
#include <automy/vehicle/UbloxReceiverClient.hxx>
#include <vnx/Input.h>
#include <vnx/Output.h>


namespace automy {
namespace vehicle {

UbloxReceiverClient::UbloxReceiverClient(const std::string& service_name)
	:	Client::Client(vnx::Hash64(service_name))
{
}

UbloxReceiverClient::UbloxReceiverClient(vnx::Hash64 service_addr)
	:	Client::Client(service_addr)
{
}

void UbloxReceiverClient::handle(const ::std::shared_ptr<const ::automy::vehicle::PPS_Signal>& sample) {
	std::shared_ptr<vnx::Binary> _argument_data = vnx::Binary::create();
	vnx::BinaryOutputStream _stream_out(_argument_data.get());
	vnx::TypeOutput _out(&_stream_out);
	const vnx::TypeCode* _type_code = vnx::get_type_code(vnx::Hash64(0x4569d59458dfbb62ull));
	{
		vnx::write(_out, sample, _type_code, _type_code->fields[0].code.data());
	}
	_out.flush();
	_argument_data->type_code = _type_code;
	vnx_request(_argument_data);
}

void UbloxReceiverClient::handle_async(const ::std::shared_ptr<const ::automy::vehicle::PPS_Signal>& sample) {
	vnx_is_async = true;
	handle(sample);
}

void UbloxReceiverClient::handle(const ::std::shared_ptr<const ::automy::vehicle::UBX_Packet>& sample) {
	std::shared_ptr<vnx::Binary> _argument_data = vnx::Binary::create();
	vnx::BinaryOutputStream _stream_out(_argument_data.get());
	vnx::TypeOutput _out(&_stream_out);
	const vnx::TypeCode* _type_code = vnx::get_type_code(vnx::Hash64(0x66612abfe9a58e6cull));
	{
		vnx::write(_out, sample, _type_code, _type_code->fields[0].code.data());
	}
	_out.flush();
	_argument_data->type_code = _type_code;
	vnx_request(_argument_data);
}

void UbloxReceiverClient::handle_async(const ::std::shared_ptr<const ::automy::vehicle::UBX_Packet>& sample) {
	vnx_is_async = true;
	handle(sample);
}


} // namespace automy
} // namespace vehicle
