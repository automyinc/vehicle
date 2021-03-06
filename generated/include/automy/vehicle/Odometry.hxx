
// AUTO GENERATED by vnxcppcodegen

#ifndef INCLUDE_automy_vehicle_Odometry_HXX_
#define INCLUDE_automy_vehicle_Odometry_HXX_

#include <automy/vehicle/package.hxx>
#include <automy/basic/Transform3D.hxx>
#include <automy/math/Vector3d.h>


namespace automy {
namespace vehicle {

class Odometry : public ::automy::basic::Transform3D {
public:
	
	::automy::math::Vector3d rotation;
	::automy::math::Vector3d position;
	::automy::math::Vector3d velocity;
	::automy::math::Vector3d angular_velocity;
	::automy::math::Vector3d acceleration;
	::automy::math::Vector3d sensor_gyro_bias;
	::vnx::float64_t speed_factor = 1;
	::vnx::bool_t is_timeout = 0;
	::std::map<::int32_t, ::automy::math::Vector3d> sensor_gyro_bias_map;
	
	typedef ::automy::basic::Transform3D Super;
	
	static const vnx::Hash64 VNX_TYPE_HASH;
	static const vnx::Hash64 VNX_CODE_HASH;
	
	vnx::Hash64 get_type_hash() const;
	const char* get_type_name() const;
	
	static std::shared_ptr<Odometry> create();
	std::shared_ptr<vnx::Value> clone() const;
	
	void read(vnx::TypeInput& _in, const vnx::TypeCode* _type_code, const uint16_t* _code);
	void write(vnx::TypeOutput& _out, const vnx::TypeCode* _type_code, const uint16_t* _code) const;
	
	void read(std::istream& _in);
	void write(std::ostream& _out) const;
	
	void accept(vnx::Visitor& _visitor) const;
	
	vnx::Object to_object() const;
	void from_object(const vnx::Object& object);
	
	friend std::ostream& operator<<(std::ostream& _out, const Odometry& _value);
	friend std::istream& operator>>(std::istream& _in, Odometry& _value);
	
	static const vnx::TypeCode* get_type_code();
	static std::shared_ptr<vnx::TypeCode> create_type_code();
	
};


} // namespace automy
} // namespace vehicle

#endif // INCLUDE_automy_vehicle_Odometry_HXX_
