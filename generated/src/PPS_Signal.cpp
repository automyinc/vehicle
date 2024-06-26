
// AUTO GENERATED by vnxcppcodegen

#include <automy/vehicle/package.hxx>
#include <automy/vehicle/PPS_Signal.hxx>
#include <vnx/Value.h>

#include <vnx/vnx.h>


namespace automy {
namespace vehicle {


const vnx::Hash64 PPS_Signal::VNX_TYPE_HASH(0x392798cc0d63fa9bull);
const vnx::Hash64 PPS_Signal::VNX_CODE_HASH(0xffe5e552a9dd0fcdull);

vnx::Hash64 PPS_Signal::get_type_hash() const {
	return VNX_TYPE_HASH;
}

std::string PPS_Signal::get_type_name() const {
	return "automy.vehicle.PPS_Signal";
}

const vnx::TypeCode* PPS_Signal::get_type_code() const {
	return automy::vehicle::vnx_native_type_code_PPS_Signal;
}

std::shared_ptr<PPS_Signal> PPS_Signal::create() {
	return std::make_shared<PPS_Signal>();
}

std::shared_ptr<vnx::Value> PPS_Signal::clone() const {
	return std::make_shared<PPS_Signal>(*this);
}

void PPS_Signal::read(vnx::TypeInput& _in, const vnx::TypeCode* _type_code, const uint16_t* _code) {
	vnx::read(_in, *this, _type_code, _code);
}

void PPS_Signal::write(vnx::TypeOutput& _out, const vnx::TypeCode* _type_code, const uint16_t* _code) const {
	vnx::write(_out, *this, _type_code, _code);
}

void PPS_Signal::accept(vnx::Visitor& _visitor) const {
	const vnx::TypeCode* _type_code = automy::vehicle::vnx_native_type_code_PPS_Signal;
	_visitor.type_begin(*_type_code);
	_visitor.type_field(_type_code->fields[0], 0); vnx::accept(_visitor, time);
	_visitor.type_field(_type_code->fields[1], 1); vnx::accept(_visitor, jitter);
	_visitor.type_end(*_type_code);
}

void PPS_Signal::write(std::ostream& _out) const {
	_out << "{\"__type\": \"automy.vehicle.PPS_Signal\"";
	_out << ", \"time\": "; vnx::write(_out, time);
	_out << ", \"jitter\": "; vnx::write(_out, jitter);
	_out << "}";
}

void PPS_Signal::read(std::istream& _in) {
	if(auto _json = vnx::read_json(_in)) {
		from_object(_json->to_object());
	}
}

vnx::Object PPS_Signal::to_object() const {
	vnx::Object _object;
	_object["__type"] = "automy.vehicle.PPS_Signal";
	_object["time"] = time;
	_object["jitter"] = jitter;
	return _object;
}

void PPS_Signal::from_object(const vnx::Object& _object) {
	for(const auto& _entry : _object.field) {
		if(_entry.first == "jitter") {
			_entry.second.to(jitter);
		} else if(_entry.first == "time") {
			_entry.second.to(time);
		}
	}
}

vnx::Variant PPS_Signal::get_field(const std::string& _name) const {
	if(_name == "time") {
		return vnx::Variant(time);
	}
	if(_name == "jitter") {
		return vnx::Variant(jitter);
	}
	return vnx::Variant();
}

void PPS_Signal::set_field(const std::string& _name, const vnx::Variant& _value) {
	if(_name == "time") {
		_value.to(time);
	} else if(_name == "jitter") {
		_value.to(jitter);
	}
}

/// \private
std::ostream& operator<<(std::ostream& _out, const PPS_Signal& _value) {
	_value.write(_out);
	return _out;
}

/// \private
std::istream& operator>>(std::istream& _in, PPS_Signal& _value) {
	_value.read(_in);
	return _in;
}

const vnx::TypeCode* PPS_Signal::static_get_type_code() {
	const vnx::TypeCode* type_code = vnx::get_type_code(VNX_TYPE_HASH);
	if(!type_code) {
		type_code = vnx::register_type_code(static_create_type_code());
	}
	return type_code;
}

std::shared_ptr<vnx::TypeCode> PPS_Signal::static_create_type_code() {
	auto type_code = std::make_shared<vnx::TypeCode>();
	type_code->name = "automy.vehicle.PPS_Signal";
	type_code->type_hash = vnx::Hash64(0x392798cc0d63fa9bull);
	type_code->code_hash = vnx::Hash64(0xffe5e552a9dd0fcdull);
	type_code->is_native = true;
	type_code->is_class = true;
	type_code->native_size = sizeof(::automy::vehicle::PPS_Signal);
	type_code->create_value = []() -> std::shared_ptr<vnx::Value> { return std::make_shared<PPS_Signal>(); };
	type_code->fields.resize(2);
	{
		auto& field = type_code->fields[0];
		field.data_size = 8;
		field.name = "time";
		field.code = {8};
	}
	{
		auto& field = type_code->fields[1];
		field.data_size = 8;
		field.name = "jitter";
		field.code = {8};
	}
	type_code->build();
	return type_code;
}

std::shared_ptr<vnx::Value> PPS_Signal::vnx_call_switch(std::shared_ptr<const vnx::Value> _method) {
	switch(_method->get_type_hash()) {
	}
	return nullptr;
}


} // namespace automy
} // namespace vehicle


namespace vnx {

void read(TypeInput& in, ::automy::vehicle::PPS_Signal& value, const TypeCode* type_code, const uint16_t* code) {
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
			vnx::read_value(_buf + _field->offset, value.jitter, _field->code.data());
		}
	}
	for(const auto* _field : type_code->ext_fields) {
		switch(_field->native_index) {
			default: vnx::skip(in, type_code, _field->code.data());
		}
	}
}

void write(TypeOutput& out, const ::automy::vehicle::PPS_Signal& value, const TypeCode* type_code, const uint16_t* code) {
	if(code && code[0] == CODE_OBJECT) {
		vnx::write(out, value.to_object(), nullptr, code);
		return;
	}
	if(!type_code || (code && code[0] == CODE_ANY)) {
		type_code = automy::vehicle::vnx_native_type_code_PPS_Signal;
		out.write_type_code(type_code);
		vnx::write_class_header<::automy::vehicle::PPS_Signal>(out);
	}
	else if(code && code[0] == CODE_STRUCT) {
		type_code = type_code->depends[code[1]];
	}
	auto* const _buf = out.write(16);
	vnx::write_value(_buf + 0, value.time);
	vnx::write_value(_buf + 8, value.jitter);
}

void read(std::istream& in, ::automy::vehicle::PPS_Signal& value) {
	value.read(in);
}

void write(std::ostream& out, const ::automy::vehicle::PPS_Signal& value) {
	value.write(out);
}

void accept(Visitor& visitor, const ::automy::vehicle::PPS_Signal& value) {
	value.accept(visitor);
}

} // vnx
