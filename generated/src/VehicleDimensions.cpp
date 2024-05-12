
// AUTO GENERATED by vnxcppcodegen

#include <automy/vehicle/package.hxx>
#include <automy/vehicle/VehicleDimensions.hxx>
#include <vnx/Value.h>

#include <vnx/vnx.h>


namespace automy {
namespace vehicle {


const vnx::Hash64 VehicleDimensions::VNX_TYPE_HASH(0xf92e12e0b29c383full);
const vnx::Hash64 VehicleDimensions::VNX_CODE_HASH(0xa0557a2f1e765539ull);

vnx::Hash64 VehicleDimensions::get_type_hash() const {
	return VNX_TYPE_HASH;
}

std::string VehicleDimensions::get_type_name() const {
	return "automy.vehicle.VehicleDimensions";
}

const vnx::TypeCode* VehicleDimensions::get_type_code() const {
	return automy::vehicle::vnx_native_type_code_VehicleDimensions;
}

std::shared_ptr<VehicleDimensions> VehicleDimensions::create() {
	return std::make_shared<VehicleDimensions>();
}

std::shared_ptr<vnx::Value> VehicleDimensions::clone() const {
	return std::make_shared<VehicleDimensions>(*this);
}

void VehicleDimensions::read(vnx::TypeInput& _in, const vnx::TypeCode* _type_code, const uint16_t* _code) {
	vnx::read(_in, *this, _type_code, _code);
}

void VehicleDimensions::write(vnx::TypeOutput& _out, const vnx::TypeCode* _type_code, const uint16_t* _code) const {
	vnx::write(_out, *this, _type_code, _code);
}

void VehicleDimensions::accept(vnx::Visitor& _visitor) const {
	const vnx::TypeCode* _type_code = automy::vehicle::vnx_native_type_code_VehicleDimensions;
	_visitor.type_begin(*_type_code);
	_visitor.type_field(_type_code->fields[0], 0); vnx::accept(_visitor, width);
	_visitor.type_field(_type_code->fields[1], 1); vnx::accept(_visitor, length);
	_visitor.type_field(_type_code->fields[2], 2); vnx::accept(_visitor, height);
	_visitor.type_field(_type_code->fields[3], 3); vnx::accept(_visitor, wheelbase);
	_visitor.type_field(_type_code->fields[4], 4); vnx::accept(_visitor, ground_offset);
	_visitor.type_field(_type_code->fields[5], 5); vnx::accept(_visitor, pos_x);
	_visitor.type_field(_type_code->fields[6], 6); vnx::accept(_visitor, neg_x);
	_visitor.type_end(*_type_code);
}

void VehicleDimensions::write(std::ostream& _out) const {
	_out << "{\"__type\": \"automy.vehicle.VehicleDimensions\"";
	_out << ", \"width\": "; vnx::write(_out, width);
	_out << ", \"length\": "; vnx::write(_out, length);
	_out << ", \"height\": "; vnx::write(_out, height);
	_out << ", \"wheelbase\": "; vnx::write(_out, wheelbase);
	_out << ", \"ground_offset\": "; vnx::write(_out, ground_offset);
	_out << ", \"pos_x\": "; vnx::write(_out, pos_x);
	_out << ", \"neg_x\": "; vnx::write(_out, neg_x);
	_out << "}";
}

void VehicleDimensions::read(std::istream& _in) {
	if(auto _json = vnx::read_json(_in)) {
		from_object(_json->to_object());
	}
}

vnx::Object VehicleDimensions::to_object() const {
	vnx::Object _object;
	_object["__type"] = "automy.vehicle.VehicleDimensions";
	_object["width"] = width;
	_object["length"] = length;
	_object["height"] = height;
	_object["wheelbase"] = wheelbase;
	_object["ground_offset"] = ground_offset;
	_object["pos_x"] = pos_x;
	_object["neg_x"] = neg_x;
	return _object;
}

void VehicleDimensions::from_object(const vnx::Object& _object) {
	for(const auto& _entry : _object.field) {
		if(_entry.first == "ground_offset") {
			_entry.second.to(ground_offset);
		} else if(_entry.first == "height") {
			_entry.second.to(height);
		} else if(_entry.first == "length") {
			_entry.second.to(length);
		} else if(_entry.first == "neg_x") {
			_entry.second.to(neg_x);
		} else if(_entry.first == "pos_x") {
			_entry.second.to(pos_x);
		} else if(_entry.first == "wheelbase") {
			_entry.second.to(wheelbase);
		} else if(_entry.first == "width") {
			_entry.second.to(width);
		}
	}
}

vnx::Variant VehicleDimensions::get_field(const std::string& _name) const {
	if(_name == "width") {
		return vnx::Variant(width);
	}
	if(_name == "length") {
		return vnx::Variant(length);
	}
	if(_name == "height") {
		return vnx::Variant(height);
	}
	if(_name == "wheelbase") {
		return vnx::Variant(wheelbase);
	}
	if(_name == "ground_offset") {
		return vnx::Variant(ground_offset);
	}
	if(_name == "pos_x") {
		return vnx::Variant(pos_x);
	}
	if(_name == "neg_x") {
		return vnx::Variant(neg_x);
	}
	return vnx::Variant();
}

void VehicleDimensions::set_field(const std::string& _name, const vnx::Variant& _value) {
	if(_name == "width") {
		_value.to(width);
	} else if(_name == "length") {
		_value.to(length);
	} else if(_name == "height") {
		_value.to(height);
	} else if(_name == "wheelbase") {
		_value.to(wheelbase);
	} else if(_name == "ground_offset") {
		_value.to(ground_offset);
	} else if(_name == "pos_x") {
		_value.to(pos_x);
	} else if(_name == "neg_x") {
		_value.to(neg_x);
	}
}

/// \private
std::ostream& operator<<(std::ostream& _out, const VehicleDimensions& _value) {
	_value.write(_out);
	return _out;
}

/// \private
std::istream& operator>>(std::istream& _in, VehicleDimensions& _value) {
	_value.read(_in);
	return _in;
}

const vnx::TypeCode* VehicleDimensions::static_get_type_code() {
	const vnx::TypeCode* type_code = vnx::get_type_code(VNX_TYPE_HASH);
	if(!type_code) {
		type_code = vnx::register_type_code(static_create_type_code());
	}
	return type_code;
}

std::shared_ptr<vnx::TypeCode> VehicleDimensions::static_create_type_code() {
	auto type_code = std::make_shared<vnx::TypeCode>();
	type_code->name = "automy.vehicle.VehicleDimensions";
	type_code->type_hash = vnx::Hash64(0xf92e12e0b29c383full);
	type_code->code_hash = vnx::Hash64(0xa0557a2f1e765539ull);
	type_code->is_native = true;
	type_code->is_class = true;
	type_code->native_size = sizeof(::automy::vehicle::VehicleDimensions);
	type_code->create_value = []() -> std::shared_ptr<vnx::Value> { return std::make_shared<VehicleDimensions>(); };
	type_code->fields.resize(7);
	{
		auto& field = type_code->fields[0];
		field.data_size = 4;
		field.name = "width";
		field.code = {9};
	}
	{
		auto& field = type_code->fields[1];
		field.data_size = 4;
		field.name = "length";
		field.code = {9};
	}
	{
		auto& field = type_code->fields[2];
		field.data_size = 4;
		field.name = "height";
		field.code = {9};
	}
	{
		auto& field = type_code->fields[3];
		field.data_size = 4;
		field.name = "wheelbase";
		field.code = {9};
	}
	{
		auto& field = type_code->fields[4];
		field.data_size = 4;
		field.name = "ground_offset";
		field.code = {9};
	}
	{
		auto& field = type_code->fields[5];
		field.data_size = 4;
		field.name = "pos_x";
		field.code = {9};
	}
	{
		auto& field = type_code->fields[6];
		field.data_size = 4;
		field.name = "neg_x";
		field.code = {9};
	}
	type_code->build();
	return type_code;
}

std::shared_ptr<vnx::Value> VehicleDimensions::vnx_call_switch(std::shared_ptr<const vnx::Value> _method) {
	switch(_method->get_type_hash()) {
	}
	return nullptr;
}


} // namespace automy
} // namespace vehicle


namespace vnx {

void read(TypeInput& in, ::automy::vehicle::VehicleDimensions& value, const TypeCode* type_code, const uint16_t* code) {
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
			vnx::read_value(_buf + _field->offset, value.width, _field->code.data());
		}
		if(const auto* const _field = type_code->field_map[1]) {
			vnx::read_value(_buf + _field->offset, value.length, _field->code.data());
		}
		if(const auto* const _field = type_code->field_map[2]) {
			vnx::read_value(_buf + _field->offset, value.height, _field->code.data());
		}
		if(const auto* const _field = type_code->field_map[3]) {
			vnx::read_value(_buf + _field->offset, value.wheelbase, _field->code.data());
		}
		if(const auto* const _field = type_code->field_map[4]) {
			vnx::read_value(_buf + _field->offset, value.ground_offset, _field->code.data());
		}
		if(const auto* const _field = type_code->field_map[5]) {
			vnx::read_value(_buf + _field->offset, value.pos_x, _field->code.data());
		}
		if(const auto* const _field = type_code->field_map[6]) {
			vnx::read_value(_buf + _field->offset, value.neg_x, _field->code.data());
		}
	}
	for(const auto* _field : type_code->ext_fields) {
		switch(_field->native_index) {
			default: vnx::skip(in, type_code, _field->code.data());
		}
	}
}

void write(TypeOutput& out, const ::automy::vehicle::VehicleDimensions& value, const TypeCode* type_code, const uint16_t* code) {
	if(code && code[0] == CODE_OBJECT) {
		vnx::write(out, value.to_object(), nullptr, code);
		return;
	}
	if(!type_code || (code && code[0] == CODE_ANY)) {
		type_code = automy::vehicle::vnx_native_type_code_VehicleDimensions;
		out.write_type_code(type_code);
		vnx::write_class_header<::automy::vehicle::VehicleDimensions>(out);
	}
	else if(code && code[0] == CODE_STRUCT) {
		type_code = type_code->depends[code[1]];
	}
	auto* const _buf = out.write(28);
	vnx::write_value(_buf + 0, value.width);
	vnx::write_value(_buf + 4, value.length);
	vnx::write_value(_buf + 8, value.height);
	vnx::write_value(_buf + 12, value.wheelbase);
	vnx::write_value(_buf + 16, value.ground_offset);
	vnx::write_value(_buf + 20, value.pos_x);
	vnx::write_value(_buf + 24, value.neg_x);
}

void read(std::istream& in, ::automy::vehicle::VehicleDimensions& value) {
	value.read(in);
}

void write(std::ostream& out, const ::automy::vehicle::VehicleDimensions& value) {
	value.write(out);
}

void accept(Visitor& visitor, const ::automy::vehicle::VehicleDimensions& value) {
	value.accept(visitor);
}

} // vnx
