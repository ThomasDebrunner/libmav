//
// Created by thomas on 05.01.23.
//

#ifndef MAV_DYNAMICMESSAGE_H
#define MAV_DYNAMICMESSAGE_H

#include <memory>
#include <array>
#include <utility>
#include <variant>
#include "MessageDefinition.h"
#include "utils.h"

namespace mav {

    using NativeVariantType = std::variant<
            std::string,
            std::vector<uint64_t>,
            std::vector<int64_t>,
            std::vector<uint32_t>,
            std::vector<int32_t>,
            std::vector<uint16_t>,
            std::vector<int16_t>,
            std::vector<uint8_t>,
            std::vector<int8_t>,
            std::vector<double>,
            std::vector<float>,
            uint64_t,
            int64_t,
            uint32_t,
            int32_t,
            uint16_t,
            int16_t,
            uint8_t,
            int8_t,
            char,
            double,
            float
    >;


    // forward declared MessageSet
    class MessageSet;

    class Message {
        friend MessageSet;
    private:
        ConnectionPartner _source_partner;
        std::array<uint8_t, MessageDefinition::MAX_MESSAGE_SIZE> _backing_memory{};
        const MessageDefinition* _message_definition;
        int _crc_offset = -1;

        explicit Message(const MessageDefinition &message_definition) :
            _message_definition(&message_definition) {
        }

        Message(const MessageDefinition &message_definition, ConnectionPartner source_partner, int crc_offset,
                std::array<uint8_t, MessageDefinition::MAX_MESSAGE_SIZE> &&backing_memory) :
                _message_definition(&message_definition),
                _source_partner(source_partner),
                _crc_offset(crc_offset),
                _backing_memory(std::move(backing_memory)) {}

        inline void _clearCRC() {
            if (_crc_offset >= 0) {
                _backing_memory[_crc_offset] = 0;
                _backing_memory[_crc_offset + 1] = 0;
                _crc_offset = -1;
            }
        }

        template <typename T>
        void _writeSingle(const Field &field, const T &v, int in_field_offset = 0) {
            // any write will potentially change the crc offset, so we invalidate it
            _clearCRC();
            // make sure that we only have simplistic base types here
            static_assert(is_any<std::decay_t<T>, short, int, long, unsigned int, unsigned long,
                    char, uint8_t, int8_t, uint16_t, int16_t, uint32_t, int32_t, uint64_t, int64_t, float, double
            >::value, "Can not set this data type to a mavlink message field.");
            // We serialize to the data type given in the field definition, not the data type used in the API.
            // This allows to use compatible data types in the API, but have them serialized to the correct data type.
            int offset = field.offset + in_field_offset;
            uint8_t* target = _backing_memory.data() + offset;

            switch (field.type.base_type) {
                case FieldType::BaseType::CHAR: return serialize(static_cast<char>(v), target);
                case FieldType::BaseType::UINT8: return serialize(static_cast<uint8_t>(v), target);
                case FieldType::BaseType::UINT16: return serialize(static_cast<uint16_t>(v), target);
                case FieldType::BaseType::UINT32: return serialize(static_cast<uint32_t>(v), target);
                case FieldType::BaseType::UINT64: return serialize(static_cast<uint64_t>(v), target);
                case FieldType::BaseType::INT8: return serialize(static_cast<int8_t>(v), target);
                case FieldType::BaseType::INT16: return serialize(static_cast<int16_t>(v), target);
                case FieldType::BaseType::INT32: return serialize(static_cast<int32_t>(v), target);
                case FieldType::BaseType::INT64: return serialize(static_cast<int64_t>(v), target);
                case FieldType::BaseType::FLOAT: return serialize(static_cast<float>(v), target);
                case FieldType::BaseType::DOUBLE: return serialize(static_cast<double>(v), target);
            }
        }

        template <typename T>
        inline T _readSingle(const Field &field, int in_field_offset = 0) const {
            const uint8_t* b_ptr = _backing_memory.data() + field.offset + in_field_offset;
            switch (field.type.base_type) {
                case FieldType::BaseType::CHAR: return static_cast<T>(deserialize<char>(b_ptr));
                case FieldType::BaseType::UINT8: return static_cast<T>(deserialize<uint8_t>(b_ptr));
                case FieldType::BaseType::UINT16: return static_cast<T>(deserialize<uint16_t>(b_ptr));
                case FieldType::BaseType::UINT32: return static_cast<T>(deserialize<uint32_t>(b_ptr));
                case FieldType::BaseType::UINT64: return static_cast<T>(deserialize<uint64_t>(b_ptr));
                case FieldType::BaseType::INT8: return static_cast<T>(deserialize<int8_t>(b_ptr));
                case FieldType::BaseType::INT16: return static_cast<T>(deserialize<int16_t>(b_ptr));
                case FieldType::BaseType::INT32: return static_cast<T>(deserialize<int32_t>(b_ptr));
                case FieldType::BaseType::INT64: return static_cast<T>(deserialize<int64_t>(b_ptr));
                case FieldType::BaseType::FLOAT: return static_cast<T>(deserialize<float>(b_ptr));
                case FieldType::BaseType::DOUBLE: return static_cast<T>(deserialize<double>(b_ptr));
            }
            throw std::runtime_error("Unknown base type"); // should never happen
        }

    public:

        static inline Message _instantiateFromMemory(const MessageDefinition &definition, ConnectionPartner source_partner,
                                          int crc_offset, std::array<uint8_t, MessageDefinition::MAX_MESSAGE_SIZE> &&backing_memory) {
            return Message{definition, source_partner, crc_offset, std::move(backing_memory)};
        }

        using _InitPairType = std::pair<const std::string, NativeVariantType>;

        template <typename MessageType>
        class _accessorType {
        private:
            const std::string &_field_name;
            MessageType &_message;
            int _array_index;
        public:
            _accessorType(const std::string &field_name, MessageType &message, int array_index) :
                _field_name(field_name), _message(message), _array_index(array_index) {}

            template <typename T>
            void operator=(const T& val) {
                _message.template set<T>(_field_name, val, _array_index);
            }

            template <typename T>
            operator T() const {
                return _message.template get<T>(_field_name, _array_index);
            }

            template <typename T>
            [[nodiscard]] T as() const {
                return _message.template get<T>(_field_name, _array_index);
            }

            _accessorType operator[](int array_index) const {
                return _accessorType{_field_name, _message, array_index};
            }

            template <typename T>
            void floatPack(T value) const {
                _message.template setAsFloatPack<T>(_field_name, value, _array_index);
            }

            template <typename T>
            T floatUnpack() const {
                return _message.template getAsFloatUnpack<T>(_field_name, _array_index);
            }
        };

        [[nodiscard]] const MessageDefinition& type() const {
            return *_message_definition;
        }

        [[nodiscard]] int id() const {
            return _message_definition->id();
        }

        [[nodiscard]] const std::string& name() const {
            return _message_definition->name();
        }

        [[nodiscard]] const Header<const uint8_t*> header() const {
            return Header<const uint8_t*>(_backing_memory.data());
        }

        [[nodiscard]] Header<uint8_t*> header() {
            return Header<uint8_t*>(_backing_memory.data());
        }

        [[nodiscard]] const ConnectionPartner& source() const {
            return _source_partner;
        }

        Message& setFromNativeTypeVariant(const std::string &field_key, const NativeVariantType &v) {
            std::visit([this, &field_key](auto&& arg) {
                this->set(field_key, arg);
            }, v);
            return *this;
        }

        Message& set(std::initializer_list<_InitPairType> init) {
            for (const auto &pair : init) {
                const auto &key = pair.first;
                setFromNativeTypeVariant(key, pair.second);
            }
            return *this;
        }

        Message& operator()(std::initializer_list<_InitPairType> init) {
            this->set(std::move(init));
            return *this;
        }

        template <typename T>
        Message& set(const std::string &field_key, const T &v, int array_index = 0) {
            auto field = _message_definition->fieldForName(field_key);

            if constexpr(is_string<T>::value) {
                setFromString(field_key, v);
            }

            else if constexpr(is_iterable<T>::value) {
                auto begin = std::begin(v);
                auto end = std::end(v);
                if ((end - begin) > field.type.size) {
                    throw std::out_of_range(StringFormat() << "Data of length " << (end - begin) <<
                        " does not fit in field with size " << field.type.size << StringFormat::end);
                }

                for (int i = 0; begin != end; (void)++begin, ++i) {
                    _writeSingle(field, *begin, (i * field.type.baseSize()));
                }
            } else {
                if (array_index < 0 || array_index >= field.type.size) {
                    throw std::out_of_range(StringFormat() << array_index <<
                        " is out of range for field " << field_key << StringFormat::end);
                }

                _writeSingle(field, v, array_index * field.type.baseSize());
            }
            return *this;
        }

        template <typename T>
        Message& setAsFloatPack(const std::string &field_key, const T &v, int array_index = 0) {
            if constexpr(is_string<T>::value) {
                throw std::runtime_error("Cannot do float unpack to a string");
            } else if constexpr(is_iterable<T>::value) {
                throw std::runtime_error("Cannot do float unpack to an iterable");
            } else {
                set<float>(field_key, floatPack<T>(v), array_index);
            }
            return *this;
        }


        Message& setFromString(const std::string &field_key, const std::string &v) {
            auto field = _message_definition->fieldForName(field_key);
            if (field.type.base_type != FieldType::BaseType::CHAR) {
                throw std::invalid_argument(StringFormat() << "Field " << field_key <<
                                                        " is not of type char" << StringFormat::end);
            }
            if (v.size() > field.type.size) {
                throw std::out_of_range(StringFormat() << "String of length " << v.size() <<
                                                       " does not fit in field with size " << field.type.size <<
                                                       StringFormat::end);
            }
            int i = 0;
            for (char c : v) {
                _writeSingle(field, c, i);
                i++;
            }
            // write a terminating zero only if there is still enough space
            if (i < field.type.size) {
                _writeSingle(field, '\0', i);
                i++;
            }
            return *this;
        }


        template <typename T>
        T get(const std::string &field_key, int array_index = 0) const {
            if constexpr(is_string<T>::value) {
                return getAsString(field_key);
            } else if constexpr(is_iterable<T>::value) {
                auto field = _message_definition->fieldForName(field_key);
                std::decay_t<T> ret_value;

                // handle std::vector: Dynamically resize for convenience
                if constexpr(std::is_same<T, std::vector<typename T::value_type>>::value) {
                    ret_value.resize(field.type.size);
                }

                if (ret_value.size() < field.type.size) {
                    throw std::out_of_range(StringFormat() << "Array of size " << field.type.size <<
                        " can not fit in return type of size " << ret_value.size() << StringFormat::end);
                }

                for (int i=0; i<field.type.size; i++) {
                    ret_value[i] = _readSingle<typename T::value_type>(field, i * field.type.baseSize());
                }
                return ret_value;
            } else {
                auto field = _message_definition->fieldForName(field_key);
                if (array_index < 0 || array_index >= field.type.size) {
                    throw std::out_of_range(StringFormat() << "Index " << array_index <<
                                                           " is out of range for field " << field_key << StringFormat::end);
                }
                return _readSingle<T>(field, array_index * field.type.baseSize());
            }
        }

        template <typename T>
        [[nodiscard]] T getAsFloatUnpack(const std::string &field_key, int array_index = 0) const {
            if constexpr(is_string<T>::value) {
                throw std::runtime_error("Cannot do float unpack to a string");
            } else if constexpr(is_iterable<T>::value) {
                throw std::runtime_error("Cannot do float unpack to an iterable");
            } else {
                return floatUnpack<T>(get<float>(field_key, array_index));
            }
        }

        [[nodiscard]] std::string getAsString(const std::string &field_key) const {
            auto field = _message_definition->fieldForName(field_key);
            if (field.type.base_type != FieldType::BaseType::CHAR) {
                throw std::invalid_argument(StringFormat() << "Field " << field_key <<
                                                        " is not of type char" << StringFormat::end);
            }
            int max_string_length = _crc_offset < 0 ?
                    field.type.size :
                    std::min(field.type.size, _crc_offset - field.offset);
            int real_string_length = strnlen(_backing_memory.data() + field.offset, max_string_length);

            return std::string{reinterpret_cast<const char*>(_backing_memory.data() + field.offset),
                               static_cast<std::string::size_type>(real_string_length)};
        }


        _accessorType<const Message> operator[](const std::string &field_name) const {
            return _accessorType<const Message>{field_name, *this, 0};
        }

        _accessorType<Message> operator[](const std::string &field_name) {
            return _accessorType<Message>{field_name, *this, 0};
        }

        [[nodiscard]] NativeVariantType getAsNativeTypeInVariant(const std::string &field_key) const {
            auto field = _message_definition->fieldForName(field_key);
            if (field.type.size <= 1) {
                switch (field.type.base_type) {
                    case FieldType::BaseType::CHAR: return get<char>(field_key);
                    case FieldType::BaseType::UINT8: return get<uint8_t>(field_key);
                    case FieldType::BaseType::UINT16: return get<uint16_t>(field_key);
                    case FieldType::BaseType::UINT32: return get<uint32_t>(field_key);
                    case FieldType::BaseType::UINT64: return get<uint64_t>(field_key);
                    case FieldType::BaseType::INT8: return get<int8_t>(field_key);
                    case FieldType::BaseType::INT16: return get<int16_t>(field_key);
                    case FieldType::BaseType::INT32: return get<int32_t>(field_key);
                    case FieldType::BaseType::INT64: return get<int64_t>(field_key);
                    case FieldType::BaseType::FLOAT: return get<float>(field_key);
                    case FieldType::BaseType::DOUBLE: return get<double>(field_key);
                }
            } else {
                switch (field.type.base_type) {
                    case FieldType::BaseType::CHAR: return get<std::string>(field_key);
                    case FieldType::BaseType::UINT8: return get<std::vector<uint8_t>>(field_key);
                    case FieldType::BaseType::UINT16: return get<std::vector<uint16_t>>(field_key);
                    case FieldType::BaseType::UINT32: return get<std::vector<uint32_t>>(field_key);
                    case FieldType::BaseType::UINT64: return get<std::vector<uint64_t>>(field_key);
                    case FieldType::BaseType::INT8: return get<std::vector<int8_t>>(field_key);
                    case FieldType::BaseType::INT16: return get<std::vector<int16_t>>(field_key);
                    case FieldType::BaseType::INT32: return get<std::vector<int32_t>>(field_key);
                    case FieldType::BaseType::INT64: return get<std::vector<int64_t>>(field_key);
                    case FieldType::BaseType::FLOAT: return get<std::vector<float>>(field_key);
                    case FieldType::BaseType::DOUBLE: return get<std::vector<double>>(field_key);
                }
            }
        }

        [[nodiscard]] uint32_t finalize(uint8_t seq, const Identifier &sender) {
            auto last_nonzero = std::find_if(_backing_memory.rend() -
                    MessageDefinition::HEADER_SIZE - _message_definition->maxPayloadSize(),
                    _backing_memory.rend(), [](const auto &v) {
                return v != 0;
            });

            int payload_size = std::max(
                    static_cast<int>(std::distance(last_nonzero, _backing_memory.rend()))
                            - MessageDefinition::HEADER_SIZE, 1);

            header().magic() = 0xFD;
            header().len() = payload_size;
            header().incompatFlags() = 0;
            header().compatFlags() = 0;
            header().seq() = seq;
            header().systemId() = sender.system_id;
            header().componentId() = sender.component_id;
            header().msgId() = _message_definition->id();

            CRC crc;
            crc.accumulate(_backing_memory.begin() + 1, _backing_memory.begin() +
                MessageDefinition::HEADER_SIZE + payload_size);
            crc.accumulate(_message_definition->crcExtra());
            _crc_offset = MessageDefinition::HEADER_SIZE + payload_size;
            serialize(crc.crc16(), _backing_memory.data() + _crc_offset);

            return MessageDefinition::HEADER_SIZE + payload_size + MessageDefinition::CHECKSUM_SIZE;
        }

        [[nodiscard]] const uint8_t* data() const {
            return _backing_memory.data();
        };
    };

} // namespace libmavlink`

#endif //MAV_DYNAMICMESSAGE_H