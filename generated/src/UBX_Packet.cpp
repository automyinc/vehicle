
// AUTO GENERATED by vnxcppcodegen

#include <automy/vehicle/package.hxx>
#include <automy/vehicle/UBX_Packet.hxx>
#include <vnx/Value.h>

#include <vnx/vnx.h>


namespace automy {
namespace vehicle {


const vnx::Hash64 UBX_Packet::VNX_TYPE_HASH(0x1a2f67e7bc19cf95ull);
const vnx::Hash64 UBX_Packet::VNX_CODE_HASH(0xacfae24daf5b9730ull);

vnx::Hash64 UBX_Packet::get_type_hash() const {
	return VNX_TYPE_HASH;
}

std::string UBX_Packet::get_type_name() const {
	return "automy.vehicle.UBX_Packet";
}

const vnx::TypeCode* UBX_Packet::get_type_code() const {
	return automy::vehicle::vnx_native_type_code_UBX_Packet;
}

std::shared_ptr<UBX_Packet> UBX_Packet::create() {
	return std::make_shared<UBX_Packet>();
}

std::shared_ptr<vnx::Value> UBX_Packet::clone() const {
	return std::make_shared<UBX_Packet>(*this);
}

void UBX_Packet::read(vnx::TypeInput& _in, const vnx::TypeCode* _type_code, const uint16_t* _code) {
	vnx::read(_in, *this, _type_code, _code);
}

void UBX_Packet::write(vnx::TypeOutput& _out, const vnx::TypeCode* _type_code, const uint16_t* _code) const {
	vnx::write(_out, *this, _type_code, _code);
}

void UBX_Packet::accept(vnx::Visitor& _visitor) const {
	const vnx::TypeCode* _type_code = automy::vehicle::vnx_native_type_code_UBX_Packet;
	_visitor.type_begin(*_type_code);
	_visitor.type_field(_type_code->fields[0], 0); vnx::accept(_visitor, time);
	_visitor.type_field(_type_code->fields[1], 1); vnx::accept(_visitor, class_id);
	_visitor.type_field(_type_code->fields[2], 2); vnx::accept(_visitor, msg_id);
	_visitor.type_field(_type_code->fields[3], 3); vnx::accept(_visitor, payload);
	_visitor.type_end(*_type_code);
}

void UBX_Packet::write(std::ostream& _out) const {
	_out << "{\"__type\": \"automy.vehicle.UBX_Packet\"";
	_out << ", \"time\": "; vnx::write(_out, time);
	_out << ", \"class_id\": "; vnx::write(_out, class_id);
	_out << ", \"msg_id\": "; vnx::write(_out, msg_id);
	_out << ", \"payload\": "; vnx::write(_out, payload);
	_out << "}";
}

void UBX_Packet::read(std::istream& _in) {
	if(auto _json = vnx::read_json(_in)) {
		from_object(_json->to_object());
	}
}

vnx::Object UBX_Packet::to_object() const {
	vnx::Object _object;
	_object["__type"] = "automy.vehicle.UBX_Packet";
	_object["time"] = time;
	_object["class_id"] = class_id;
	_object["msg_id"] = msg_id;
	_object["payload"] = payload;
	return _object;
}

void UBX_Packet::from_object(const vnx::Object& _object) {
	for(const auto& _entry : _object.field) {
		if(_entry.first == "class_id") {
			_entry.second.to(class_id);
		} else if(_entry.first == "msg_id") {
			_entry.second.to(msg_id);
		} else if(_entry.first == "payload") {
			_entry.second.to(payload);
		} else if(_entry.first == "time") {
			_entry.second.to(time);
		}
	}
}

vnx::Variant UBX_Packet::get_field(const std::string& _name) const {
	if(_name == "time") {
		return vnx::Variant(time);
	}
	if(_name == "class_id") {
		return vnx::Variant(class_id);
	}
	if(_name == "msg_id") {
		return vnx::Variant(msg_id);
	}
	if(_name == "payload") {
		return vnx::Variant(payload);
	}
	return vnx::Variant();
}

void UBX_Packet::set_field(const std::string& _name, const vnx::Variant& _value) {
	if(_name == "time") {
		_value.to(time);
	} else if(_name == "class_id") {
		_value.to(class_id);
	} else if(_name == "msg_id") {
		_value.to(msg_id);
	} else if(_name == "payload") {
		_value.to(payload);
	}
}

/// \private
std::ostream& operator<<(std::ostream& _out, const UBX_Packet& _value) {
	_value.write(_out);
	return _out;
}

/// \private
std::istream& operator>>(std::istream& _in, UBX_Packet& _value) {
	_value.read(_in);
	return _in;
}

const vnx::TypeCode* UBX_Packet::static_get_type_code() {
	const vnx::TypeCode* type_code = vnx::get_type_code(VNX_TYPE_HASH);
	if(!type_code) {
		type_code = vnx::register_type_code(static_create_type_code());
	}
	return type_code;
}

std::shared_ptr<vnx::TypeCode> UBX_Packet::static_create_type_code() {
	auto type_code = std::make_shared<vnx::TypeCode>();
	type_code->name = "automy.vehicle.UBX_Packet";
	type_code->type_hash = vnx::Hash64(0x1a2f67e7bc19cf95ull);
	type_code->code_hash = vnx::Hash64(0xacfae24daf5b9730ull);
	type_code->is_native = true;
	type_code->is_class = true;
	type_code->native_size = sizeof(::automy::vehicle::UBX_Packet);
	type_code->create_value = []() -> std::shared_ptr<vnx::Value> { return std::make_shared<UBX_Packet>(); };
	type_code->fields.resize(4);
	{
		auto& field = type_code->fields[0];
		field.data_size = 8;
		field.name = "time";
		field.code = {8};
	}
	{
		auto& field = type_code->fields[1];
		field.data_size = 1;
		field.name = "class_id";
		field.code = {1};
	}
	{
		auto& field = type_code->fields[2];
		field.data_size = 1;
		field.name = "msg_id";
		field.code = {1};
	}
	{
		auto& field = type_code->fields[3];
		field.is_extended = true;
		field.name = "payload";
		field.code = {12, 1};
	}
	type_code->build();
	return type_code;
}

std::shared_ptr<vnx::Value> UBX_Packet::vnx_call_switch(std::shared_ptr<const vnx::Value> _method) {
	switch(_method->get_type_hash()) {
	}
	return nullptr;
}


} // namespace automy
} // namespace vehicle


namespace vnx {

void read(TypeInput& in, ::automy::vehicle::UBX_Packet& value, const TypeCode* type_code, const uint16_t* code) {
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
			vnx::read_value(_buf + _field->offset, value.class_id, _field->code.data());
		}
		if(const auto* const _field = type_code->field_map[2]) {
			vnx::read_value(_buf + _field->offset, value.msg_id, _field->code.data());
		}
	}
	for(const auto* _field : type_code->ext_fields) {
		switch(_field->native_index) {
			case 3: vnx::read(in, value.payload, type_code, _field->code.data()); break;
			default: vnx::skip(in, type_code, _field->code.data());
		}
	}
}

void write(TypeOutput& out, const ::automy::vehicle::UBX_Packet& value, const TypeCode* type_code, const uint16_t* code) {
	if(code && code[0] == CODE_OBJECT) {
		vnx::write(out, value.to_object(), nullptr, code);
		return;
	}
	if(!type_code || (code && code[0] == CODE_ANY)) {
		type_code = automy::vehicle::vnx_native_type_code_UBX_Packet;
		out.write_type_code(type_code);
		vnx::write_class_header<::automy::vehicle::UBX_Packet>(out);
	}
	else if(code && code[0] == CODE_STRUCT) {
		type_code = type_code->depends[code[1]];
	}
	auto* const _buf = out.write(10);
	vnx::write_value(_buf + 0, value.time);
	vnx::write_value(_buf + 8, value.class_id);
	vnx::write_value(_buf + 9, value.msg_id);
	vnx::write(out, value.payload, type_code, type_code->fields[3].code.data());
}

void read(std::istream& in, ::automy::vehicle::UBX_Packet& value) {
	value.read(in);
}

void write(std::ostream& out, const ::automy::vehicle::UBX_Packet& value) {
	value.write(out);
}

void accept(Visitor& visitor, const ::automy::vehicle::UBX_Packet& value) {
	value.accept(visitor);
}

} // vnx
