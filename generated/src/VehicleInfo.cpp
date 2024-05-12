
// AUTO GENERATED by vnxcppcodegen

#include <automy/vehicle/package.hxx>
#include <automy/vehicle/VehicleInfo.hxx>
#include <automy/vehicle/VehicleDimensions.hxx>
#include <vnx/Value.h>

#include <vnx/vnx.h>


namespace automy {
namespace vehicle {


const vnx::Hash64 VehicleInfo::VNX_TYPE_HASH(0xe1eb64ab47c30856ull);
const vnx::Hash64 VehicleInfo::VNX_CODE_HASH(0x25ce9299074d8a9cull);

vnx::Hash64 VehicleInfo::get_type_hash() const {
	return VNX_TYPE_HASH;
}

std::string VehicleInfo::get_type_name() const {
	return "automy.vehicle.VehicleInfo";
}

const vnx::TypeCode* VehicleInfo::get_type_code() const {
	return automy::vehicle::vnx_native_type_code_VehicleInfo;
}

std::shared_ptr<VehicleInfo> VehicleInfo::create() {
	return std::make_shared<VehicleInfo>();
}

std::shared_ptr<vnx::Value> VehicleInfo::clone() const {
	return std::make_shared<VehicleInfo>(*this);
}

void VehicleInfo::read(vnx::TypeInput& _in, const vnx::TypeCode* _type_code, const uint16_t* _code) {
	vnx::read(_in, *this, _type_code, _code);
}

void VehicleInfo::write(vnx::TypeOutput& _out, const vnx::TypeCode* _type_code, const uint16_t* _code) const {
	vnx::write(_out, *this, _type_code, _code);
}

void VehicleInfo::accept(vnx::Visitor& _visitor) const {
	const vnx::TypeCode* _type_code = automy::vehicle::vnx_native_type_code_VehicleInfo;
	_visitor.type_begin(*_type_code);
	_visitor.type_field(_type_code->fields[0], 0); vnx::accept(_visitor, time);
	_visitor.type_field(_type_code->fields[1], 1); vnx::accept(_visitor, speed);
	_visitor.type_field(_type_code->fields[2], 2); vnx::accept(_visitor, steering_wheel_angle);
	_visitor.type_field(_type_code->fields[3], 3); vnx::accept(_visitor, steering_wheel_velocity);
	_visitor.type_field(_type_code->fields[4], 4); vnx::accept(_visitor, dimensions);
	_visitor.type_field(_type_code->fields[5], 5); vnx::accept(_visitor, steering_ratio);
	_visitor.type_end(*_type_code);
}

void VehicleInfo::write(std::ostream& _out) const {
	_out << "{\"__type\": \"automy.vehicle.VehicleInfo\"";
	_out << ", \"time\": "; vnx::write(_out, time);
	_out << ", \"speed\": "; vnx::write(_out, speed);
	_out << ", \"steering_wheel_angle\": "; vnx::write(_out, steering_wheel_angle);
	_out << ", \"steering_wheel_velocity\": "; vnx::write(_out, steering_wheel_velocity);
	_out << ", \"dimensions\": "; vnx::write(_out, dimensions);
	_out << ", \"steering_ratio\": "; vnx::write(_out, steering_ratio);
	_out << "}";
}

void VehicleInfo::read(std::istream& _in) {
	if(auto _json = vnx::read_json(_in)) {
		from_object(_json->to_object());
	}
}

vnx::Object VehicleInfo::to_object() const {
	vnx::Object _object;
	_object["__type"] = "automy.vehicle.VehicleInfo";
	_object["time"] = time;
	_object["speed"] = speed;
	_object["steering_wheel_angle"] = steering_wheel_angle;
	_object["steering_wheel_velocity"] = steering_wheel_velocity;
	_object["dimensions"] = dimensions;
	_object["steering_ratio"] = steering_ratio;
	return _object;
}

void VehicleInfo::from_object(const vnx::Object& _object) {
	for(const auto& _entry : _object.field) {
		if(_entry.first == "dimensions") {
			_entry.second.to(dimensions);
		} else if(_entry.first == "speed") {
			_entry.second.to(speed);
		} else if(_entry.first == "steering_ratio") {
			_entry.second.to(steering_ratio);
		} else if(_entry.first == "steering_wheel_angle") {
			_entry.second.to(steering_wheel_angle);
		} else if(_entry.first == "steering_wheel_velocity") {
			_entry.second.to(steering_wheel_velocity);
		} else if(_entry.first == "time") {
			_entry.second.to(time);
		}
	}
}

vnx::Variant VehicleInfo::get_field(const std::string& _name) const {
	if(_name == "time") {
		return vnx::Variant(time);
	}
	if(_name == "speed") {
		return vnx::Variant(speed);
	}
	if(_name == "steering_wheel_angle") {
		return vnx::Variant(steering_wheel_angle);
	}
	if(_name == "steering_wheel_velocity") {
		return vnx::Variant(steering_wheel_velocity);
	}
	if(_name == "dimensions") {
		return vnx::Variant(dimensions);
	}
	if(_name == "steering_ratio") {
		return vnx::Variant(steering_ratio);
	}
	return vnx::Variant();
}

void VehicleInfo::set_field(const std::string& _name, const vnx::Variant& _value) {
	if(_name == "time") {
		_value.to(time);
	} else if(_name == "speed") {
		_value.to(speed);
	} else if(_name == "steering_wheel_angle") {
		_value.to(steering_wheel_angle);
	} else if(_name == "steering_wheel_velocity") {
		_value.to(steering_wheel_velocity);
	} else if(_name == "dimensions") {
		_value.to(dimensions);
	} else if(_name == "steering_ratio") {
		_value.to(steering_ratio);
	}
}

/// \private
std::ostream& operator<<(std::ostream& _out, const VehicleInfo& _value) {
	_value.write(_out);
	return _out;
}

/// \private
std::istream& operator>>(std::istream& _in, VehicleInfo& _value) {
	_value.read(_in);
	return _in;
}

const vnx::TypeCode* VehicleInfo::static_get_type_code() {
	const vnx::TypeCode* type_code = vnx::get_type_code(VNX_TYPE_HASH);
	if(!type_code) {
		type_code = vnx::register_type_code(static_create_type_code());
	}
	return type_code;
}

std::shared_ptr<vnx::TypeCode> VehicleInfo::static_create_type_code() {
	auto type_code = std::make_shared<vnx::TypeCode>();
	type_code->name = "automy.vehicle.VehicleInfo";
	type_code->type_hash = vnx::Hash64(0xe1eb64ab47c30856ull);
	type_code->code_hash = vnx::Hash64(0x25ce9299074d8a9cull);
	type_code->is_native = true;
	type_code->is_class = true;
	type_code->native_size = sizeof(::automy::vehicle::VehicleInfo);
	type_code->create_value = []() -> std::shared_ptr<vnx::Value> { return std::make_shared<VehicleInfo>(); };
	type_code->depends.resize(1);
	type_code->depends[0] = ::automy::vehicle::VehicleDimensions::static_get_type_code();
	type_code->fields.resize(6);
	{
		auto& field = type_code->fields[0];
		field.data_size = 8;
		field.name = "time";
		field.code = {8};
	}
	{
		auto& field = type_code->fields[1];
		field.data_size = 8;
		field.name = "speed";
		field.code = {10};
	}
	{
		auto& field = type_code->fields[2];
		field.data_size = 8;
		field.name = "steering_wheel_angle";
		field.code = {10};
	}
	{
		auto& field = type_code->fields[3];
		field.data_size = 8;
		field.name = "steering_wheel_velocity";
		field.code = {10};
	}
	{
		auto& field = type_code->fields[4];
		field.is_extended = true;
		field.name = "dimensions";
		field.code = {19, 0};
	}
	{
		auto& field = type_code->fields[5];
		field.data_size = 4;
		field.name = "steering_ratio";
		field.code = {9};
	}
	type_code->build();
	return type_code;
}

std::shared_ptr<vnx::Value> VehicleInfo::vnx_call_switch(std::shared_ptr<const vnx::Value> _method) {
	switch(_method->get_type_hash()) {
	}
	return nullptr;
}


} // namespace automy
} // namespace vehicle


namespace vnx {

void read(TypeInput& in, ::automy::vehicle::VehicleInfo& value, const TypeCode* type_code, const uint16_t* code) {
	if(code) {
		switch(code[0]) {
			case CODE_OBJECT:
			case CODE_ALT_OBJECT: {
				Object tmp;
				vnx::read(in, tmp, type_code, code);
				value.from_object(tmp);
				return;
			}
			case CODE_DYNAMIC:
			case CODE_ALT_DYNAMIC:
				vnx::read_dynamic(in, value);
				return;
		}
	}
	if(!type_code) {
		vnx::skip(in, type_code, code);
		return;
	}
	if(code) {
		switch(code[0]) {
			case CODE_STRUCT: type_code = type_code->depends[code[1]]; break;
			case CODE_ALT_STRUCT: type_code = type_code->depends[vnx::flip_bytes(code[1])]; break;
			default: {
				vnx::skip(in, type_code, code);
				return;
			}
		}
	}
	const auto* const _buf = in.read(type_code->total_field_size);
	if(type_code->is_matched) {
		if(const auto* const _field = type_code->field_map[0]) {
			vnx::read_value(_buf + _field->offset, value.time, _field->code.data());
		}
		if(const auto* const _field = type_code->field_map[1]) {
			vnx::read_value(_buf + _field->offset, value.speed, _field->code.data());
		}
		if(const auto* const _field = type_code->field_map[2]) {
			vnx::read_value(_buf + _field->offset, value.steering_wheel_angle, _field->code.data());
		}
		if(const auto* const _field = type_code->field_map[3]) {
			vnx::read_value(_buf + _field->offset, value.steering_wheel_velocity, _field->code.data());
		}
		if(const auto* const _field = type_code->field_map[5]) {
			vnx::read_value(_buf + _field->offset, value.steering_ratio, _field->code.data());
		}
	}
	for(const auto* _field : type_code->ext_fields) {
		switch(_field->native_index) {
			case 4: vnx::read(in, value.dimensions, type_code, _field->code.data()); break;
			default: vnx::skip(in, type_code, _field->code.data());
		}
	}
}

void write(TypeOutput& out, const ::automy::vehicle::VehicleInfo& value, const TypeCode* type_code, const uint16_t* code) {
	if(code && code[0] == CODE_OBJECT) {
		vnx::write(out, value.to_object(), nullptr, code);
		return;
	}
	if(!type_code || (code && code[0] == CODE_ANY)) {
		type_code = automy::vehicle::vnx_native_type_code_VehicleInfo;
		out.write_type_code(type_code);
		vnx::write_class_header<::automy::vehicle::VehicleInfo>(out);
	}
	else if(code && code[0] == CODE_STRUCT) {
		type_code = type_code->depends[code[1]];
	}
	auto* const _buf = out.write(36);
	vnx::write_value(_buf + 0, value.time);
	vnx::write_value(_buf + 8, value.speed);
	vnx::write_value(_buf + 16, value.steering_wheel_angle);
	vnx::write_value(_buf + 24, value.steering_wheel_velocity);
	vnx::write_value(_buf + 32, value.steering_ratio);
	vnx::write(out, value.dimensions, type_code, type_code->fields[4].code.data());
}

void read(std::istream& in, ::automy::vehicle::VehicleInfo& value) {
	value.read(in);
}

void write(std::ostream& out, const ::automy::vehicle::VehicleInfo& value) {
	value.write(out);
}

void accept(Visitor& visitor, const ::automy::vehicle::VehicleInfo& value) {
	value.accept(visitor);
}

} // vnx
