
// AUTO GENERATED by vnxcppcodegen

#include <automy/vehicle/package.hxx>
#include <automy/vehicle/Odometry.hxx>
#include <vnx/Input.h>
#include <vnx/Output.h>
#include <vnx/Visitor.h>
#include <vnx/Object.h>
#include <vnx/Struct.h>


namespace automy {
namespace vehicle {


const vnx::Hash64 Odometry::VNX_TYPE_HASH(0x543083a351666ea6ull);
const vnx::Hash64 Odometry::VNX_CODE_HASH(0x7f0c2dbc1dcfc436ull);

vnx::Hash64 Odometry::get_type_hash() const {
	return VNX_TYPE_HASH;
}

const char* Odometry::get_type_name() const {
	return "automy.vehicle.Odometry";
}

std::shared_ptr<Odometry> Odometry::create() {
	return std::make_shared<Odometry>();
}

std::shared_ptr<vnx::Value> Odometry::clone() const {
	return std::make_shared<Odometry>(*this);
}

void Odometry::read(vnx::TypeInput& _in, const vnx::TypeCode* _type_code, const uint16_t* _code) {
	vnx::read(_in, *this, _type_code, _code);
}

void Odometry::write(vnx::TypeOutput& _out, const vnx::TypeCode* _type_code, const uint16_t* _code) const {
	vnx::write(_out, *this, _type_code, _code);
}

void Odometry::accept(vnx::Visitor& _visitor) const {
	const vnx::TypeCode* _type_code = get_type_code();
	_visitor.type_begin(*_type_code);
	_visitor.type_field(_type_code->fields[0], 0); vnx::accept(_visitor, time);
	_visitor.type_field(_type_code->fields[1], 1); vnx::accept(_visitor, matrix);
	_visitor.type_field(_type_code->fields[2], 2); vnx::accept(_visitor, rotation);
	_visitor.type_field(_type_code->fields[3], 3); vnx::accept(_visitor, position);
	_visitor.type_field(_type_code->fields[4], 4); vnx::accept(_visitor, velocity);
	_visitor.type_field(_type_code->fields[5], 5); vnx::accept(_visitor, angular_velocity);
	_visitor.type_field(_type_code->fields[6], 6); vnx::accept(_visitor, acceleration);
	_visitor.type_field(_type_code->fields[7], 7); vnx::accept(_visitor, sensor_gyro_bias);
	_visitor.type_field(_type_code->fields[8], 8); vnx::accept(_visitor, speed_factor);
	_visitor.type_field(_type_code->fields[9], 9); vnx::accept(_visitor, is_timeout);
	_visitor.type_field(_type_code->fields[10], 10); vnx::accept(_visitor, sensor_gyro_bias_map);
	_visitor.type_end(*_type_code);
}

void Odometry::write(std::ostream& _out) const {
	_out << "{";
	_out << "\"time\": "; vnx::write(_out, time);
	_out << ", \"matrix\": "; vnx::write(_out, matrix);
	_out << ", \"rotation\": "; vnx::write(_out, rotation);
	_out << ", \"position\": "; vnx::write(_out, position);
	_out << ", \"velocity\": "; vnx::write(_out, velocity);
	_out << ", \"angular_velocity\": "; vnx::write(_out, angular_velocity);
	_out << ", \"acceleration\": "; vnx::write(_out, acceleration);
	_out << ", \"sensor_gyro_bias\": "; vnx::write(_out, sensor_gyro_bias);
	_out << ", \"speed_factor\": "; vnx::write(_out, speed_factor);
	_out << ", \"is_timeout\": "; vnx::write(_out, is_timeout);
	_out << ", \"sensor_gyro_bias_map\": "; vnx::write(_out, sensor_gyro_bias_map);
	_out << "}";
}

void Odometry::read(std::istream& _in) {
	std::map<std::string, std::string> _object;
	vnx::read_object(_in, _object);
	for(const auto& _entry : _object) {
		if(_entry.first == "acceleration") {
			vnx::from_string(_entry.second, acceleration);
		} else if(_entry.first == "angular_velocity") {
			vnx::from_string(_entry.second, angular_velocity);
		} else if(_entry.first == "is_timeout") {
			vnx::from_string(_entry.second, is_timeout);
		} else if(_entry.first == "matrix") {
			vnx::from_string(_entry.second, matrix);
		} else if(_entry.first == "position") {
			vnx::from_string(_entry.second, position);
		} else if(_entry.first == "rotation") {
			vnx::from_string(_entry.second, rotation);
		} else if(_entry.first == "sensor_gyro_bias") {
			vnx::from_string(_entry.second, sensor_gyro_bias);
		} else if(_entry.first == "sensor_gyro_bias_map") {
			vnx::from_string(_entry.second, sensor_gyro_bias_map);
		} else if(_entry.first == "speed_factor") {
			vnx::from_string(_entry.second, speed_factor);
		} else if(_entry.first == "time") {
			vnx::from_string(_entry.second, time);
		} else if(_entry.first == "velocity") {
			vnx::from_string(_entry.second, velocity);
		}
	}
}

vnx::Object Odometry::to_object() const {
	vnx::Object _object;
	_object["time"] = time;
	_object["matrix"] = matrix;
	_object["rotation"] = rotation;
	_object["position"] = position;
	_object["velocity"] = velocity;
	_object["angular_velocity"] = angular_velocity;
	_object["acceleration"] = acceleration;
	_object["sensor_gyro_bias"] = sensor_gyro_bias;
	_object["speed_factor"] = speed_factor;
	_object["is_timeout"] = is_timeout;
	_object["sensor_gyro_bias_map"] = sensor_gyro_bias_map;
	return _object;
}

void Odometry::from_object(const vnx::Object& _object) {
	for(const auto& _entry : _object.field) {
		if(_entry.first == "acceleration") {
			_entry.second.to(acceleration);
		} else if(_entry.first == "angular_velocity") {
			_entry.second.to(angular_velocity);
		} else if(_entry.first == "is_timeout") {
			_entry.second.to(is_timeout);
		} else if(_entry.first == "matrix") {
			_entry.second.to(matrix);
		} else if(_entry.first == "position") {
			_entry.second.to(position);
		} else if(_entry.first == "rotation") {
			_entry.second.to(rotation);
		} else if(_entry.first == "sensor_gyro_bias") {
			_entry.second.to(sensor_gyro_bias);
		} else if(_entry.first == "sensor_gyro_bias_map") {
			_entry.second.to(sensor_gyro_bias_map);
		} else if(_entry.first == "speed_factor") {
			_entry.second.to(speed_factor);
		} else if(_entry.first == "time") {
			_entry.second.to(time);
		} else if(_entry.first == "velocity") {
			_entry.second.to(velocity);
		}
	}
}

/// \private
std::ostream& operator<<(std::ostream& _out, const Odometry& _value) {
	_value.write(_out);
	return _out;
}

/// \private
std::istream& operator>>(std::istream& _in, Odometry& _value) {
	_value.read(_in);
	return _in;
}

const vnx::TypeCode* Odometry::get_type_code() {
	const vnx::TypeCode* type_code = vnx::get_type_code(vnx::Hash64(0x543083a351666ea6ull));
	if(!type_code) {
		type_code = vnx::register_type_code(create_type_code());
	}
	return type_code;
}

std::shared_ptr<vnx::TypeCode> Odometry::create_type_code() {
	std::shared_ptr<vnx::TypeCode> type_code = std::make_shared<vnx::TypeCode>(true);
	type_code->name = "automy.vehicle.Odometry";
	type_code->type_hash = vnx::Hash64(0x543083a351666ea6ull);
	type_code->code_hash = vnx::Hash64(0x7f0c2dbc1dcfc436ull);
	type_code->is_class = true;
	type_code->parents.resize(1);
	type_code->parents[0] = ::automy::basic::Transform3D::get_type_code();
	type_code->create_value = []() -> std::shared_ptr<vnx::Value> { return std::make_shared<Odometry>(); };
	type_code->fields.resize(11);
	{
		vnx::TypeField& field = type_code->fields[0];
		field.name = "time";
		field.code = {8};
	}
	{
		vnx::TypeField& field = type_code->fields[1];
		field.is_extended = true;
		field.name = "matrix";
		field.code = {21, 2, 4, 4, 10};
	}
	{
		vnx::TypeField& field = type_code->fields[2];
		field.is_extended = true;
		field.name = "rotation";
		field.code = {21, 2, 3, 1, 10};
	}
	{
		vnx::TypeField& field = type_code->fields[3];
		field.is_extended = true;
		field.name = "position";
		field.code = {21, 2, 3, 1, 10};
	}
	{
		vnx::TypeField& field = type_code->fields[4];
		field.is_extended = true;
		field.name = "velocity";
		field.code = {21, 2, 3, 1, 10};
	}
	{
		vnx::TypeField& field = type_code->fields[5];
		field.is_extended = true;
		field.name = "angular_velocity";
		field.code = {21, 2, 3, 1, 10};
	}
	{
		vnx::TypeField& field = type_code->fields[6];
		field.is_extended = true;
		field.name = "acceleration";
		field.code = {21, 2, 3, 1, 10};
	}
	{
		vnx::TypeField& field = type_code->fields[7];
		field.is_extended = true;
		field.name = "sensor_gyro_bias";
		field.code = {21, 2, 3, 1, 10};
	}
	{
		vnx::TypeField& field = type_code->fields[8];
		field.name = "speed_factor";
		field.value = vnx::to_string(1);
		field.code = {10};
	}
	{
		vnx::TypeField& field = type_code->fields[9];
		field.name = "is_timeout";
		field.code = {1};
	}
	{
		vnx::TypeField& field = type_code->fields[10];
		field.is_extended = true;
		field.name = "sensor_gyro_bias_map";
		field.code = {13, 3, 7, 21, 2, 3, 1, 10};
	}
	type_code->build();
	return type_code;
}


} // namespace automy
} // namespace vehicle


namespace vnx {

void read(TypeInput& in, ::automy::vehicle::Odometry& value, const TypeCode* type_code, const uint16_t* code) {
	if(!type_code) {
		throw std::logic_error("read(): type_code == 0");
	}
	if(code) {
		switch(code[0]) {
			case CODE_STRUCT: type_code = type_code->depends[code[1]]; break;
			case CODE_ALT_STRUCT: type_code = type_code->depends[vnx::flip_bytes(code[1])]; break;
			default: vnx::skip(in, type_code, code); return;
		}
	}
	const char* const _buf = in.read(type_code->total_field_size);
	if(type_code->is_matched) {
		{
			const vnx::TypeField* const _field = type_code->field_map[0];
			if(_field) {
				vnx::read_value(_buf + _field->offset, value.time, _field->code.data());
			}
		}
		{
			const vnx::TypeField* const _field = type_code->field_map[8];
			if(_field) {
				vnx::read_value(_buf + _field->offset, value.speed_factor, _field->code.data());
			}
		}
		{
			const vnx::TypeField* const _field = type_code->field_map[9];
			if(_field) {
				vnx::read_value(_buf + _field->offset, value.is_timeout, _field->code.data());
			}
		}
	}
	for(const vnx::TypeField* _field : type_code->ext_fields) {
		switch(_field->native_index) {
			case 1: vnx::read(in, value.matrix, type_code, _field->code.data()); break;
			case 2: vnx::read(in, value.rotation, type_code, _field->code.data()); break;
			case 3: vnx::read(in, value.position, type_code, _field->code.data()); break;
			case 4: vnx::read(in, value.velocity, type_code, _field->code.data()); break;
			case 5: vnx::read(in, value.angular_velocity, type_code, _field->code.data()); break;
			case 6: vnx::read(in, value.acceleration, type_code, _field->code.data()); break;
			case 7: vnx::read(in, value.sensor_gyro_bias, type_code, _field->code.data()); break;
			case 10: vnx::read(in, value.sensor_gyro_bias_map, type_code, _field->code.data()); break;
			default: vnx::skip(in, type_code, _field->code.data());
		}
	}
}

void write(TypeOutput& out, const ::automy::vehicle::Odometry& value, const TypeCode* type_code, const uint16_t* code) {
	if(!type_code || (code && code[0] == CODE_ANY)) {
		type_code = vnx::write_type_code<::automy::vehicle::Odometry>(out);
		vnx::write_class_header<::automy::vehicle::Odometry>(out);
	}
	if(code && code[0] == CODE_STRUCT) {
		type_code = type_code->depends[code[1]];
	}
	char* const _buf = out.write(17);
	vnx::write_value(_buf + 0, value.time);
	vnx::write_value(_buf + 8, value.speed_factor);
	vnx::write_value(_buf + 16, value.is_timeout);
	vnx::write(out, value.matrix, type_code, type_code->fields[1].code.data());
	vnx::write(out, value.rotation, type_code, type_code->fields[2].code.data());
	vnx::write(out, value.position, type_code, type_code->fields[3].code.data());
	vnx::write(out, value.velocity, type_code, type_code->fields[4].code.data());
	vnx::write(out, value.angular_velocity, type_code, type_code->fields[5].code.data());
	vnx::write(out, value.acceleration, type_code, type_code->fields[6].code.data());
	vnx::write(out, value.sensor_gyro_bias, type_code, type_code->fields[7].code.data());
	vnx::write(out, value.sensor_gyro_bias_map, type_code, type_code->fields[10].code.data());
}

void read(std::istream& in, ::automy::vehicle::Odometry& value) {
	value.read(in);
}

void write(std::ostream& out, const ::automy::vehicle::Odometry& value) {
	value.write(out);
}

void accept(Visitor& visitor, const ::automy::vehicle::Odometry& value) {
	value.accept(visitor);
}

} // vnx
