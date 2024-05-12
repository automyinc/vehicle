
// AUTO GENERATED by vnxcppcodegen

#ifndef INCLUDE_automy_vehicle_GPS_Info_HXX_
#define INCLUDE_automy_vehicle_GPS_Info_HXX_

#include <automy/vehicle/package.hxx>
#include <automy/math/Vector3d.hpp>
#include <vnx/Value.h>


namespace automy {
namespace vehicle {

class AUTOMY_VEHICLE_EXPORT GPS_Info : public ::vnx::Value {
public:
	
	int64_t time = 0;
	int64_t gps_time = 0;
	uint32_t time_of_week = 0;
	vnx::bool_t have_fix = 0;
	vnx::bool_t have_pps = 0;
	vnx::float64_t latitude = 0;
	vnx::float64_t longitude = 0;
	vnx::float64_t height = 0;
	vnx::float64_t heading = 0;
	vnx::float64_t speed = 0;
	vnx::float64_t vel_north = 0;
	vnx::float64_t vel_east = 0;
	vnx::float64_t vel_up = 0;
	vnx::float32_t hdop = -1;
	vnx::float32_t vdop = -1;
	int32_t num_sats = -1;
	::automy::math::Vector3d antenna_pos;
	
	typedef ::vnx::Value Super;
	
	static const vnx::Hash64 VNX_TYPE_HASH;
	static const vnx::Hash64 VNX_CODE_HASH;
	
	static constexpr uint64_t VNX_TYPE_ID = 0xbd0d89a3f33315e3ull;
	
	GPS_Info() {}
	
	vnx::Hash64 get_type_hash() const override;
	std::string get_type_name() const override;
	const vnx::TypeCode* get_type_code() const override;
	
	static std::shared_ptr<GPS_Info> create();
	std::shared_ptr<vnx::Value> clone() const override;
	
	void read(vnx::TypeInput& _in, const vnx::TypeCode* _type_code, const uint16_t* _code) override;
	void write(vnx::TypeOutput& _out, const vnx::TypeCode* _type_code, const uint16_t* _code) const override;
	
	void read(std::istream& _in) override;
	void write(std::ostream& _out) const override;
	
	template<typename T>
	void accept_generic(T& _visitor) const;
	void accept(vnx::Visitor& _visitor) const override;
	
	vnx::Object to_object() const override;
	void from_object(const vnx::Object& object) override;
	
	vnx::Variant get_field(const std::string& name) const override;
	void set_field(const std::string& name, const vnx::Variant& value) override;
	
	friend std::ostream& operator<<(std::ostream& _out, const GPS_Info& _value);
	friend std::istream& operator>>(std::istream& _in, GPS_Info& _value);
	
	static const vnx::TypeCode* static_get_type_code();
	static std::shared_ptr<vnx::TypeCode> static_create_type_code();
	
protected:
	std::shared_ptr<vnx::Value> vnx_call_switch(std::shared_ptr<const vnx::Value> _method) override;
	
};

template<typename T>
void GPS_Info::accept_generic(T& _visitor) const {
	_visitor.template type_begin<GPS_Info>(17);
	_visitor.type_field("time", 0); _visitor.accept(time);
	_visitor.type_field("gps_time", 1); _visitor.accept(gps_time);
	_visitor.type_field("time_of_week", 2); _visitor.accept(time_of_week);
	_visitor.type_field("have_fix", 3); _visitor.accept(have_fix);
	_visitor.type_field("have_pps", 4); _visitor.accept(have_pps);
	_visitor.type_field("latitude", 5); _visitor.accept(latitude);
	_visitor.type_field("longitude", 6); _visitor.accept(longitude);
	_visitor.type_field("height", 7); _visitor.accept(height);
	_visitor.type_field("heading", 8); _visitor.accept(heading);
	_visitor.type_field("speed", 9); _visitor.accept(speed);
	_visitor.type_field("vel_north", 10); _visitor.accept(vel_north);
	_visitor.type_field("vel_east", 11); _visitor.accept(vel_east);
	_visitor.type_field("vel_up", 12); _visitor.accept(vel_up);
	_visitor.type_field("hdop", 13); _visitor.accept(hdop);
	_visitor.type_field("vdop", 14); _visitor.accept(vdop);
	_visitor.type_field("num_sats", 15); _visitor.accept(num_sats);
	_visitor.type_field("antenna_pos", 16); _visitor.accept(antenna_pos);
	_visitor.template type_end<GPS_Info>(17);
}


} // namespace automy
} // namespace vehicle


namespace vnx {

} // vnx

#endif // INCLUDE_automy_vehicle_GPS_Info_HXX_
