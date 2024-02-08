// Copyright 2016-2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <fastcdr/exceptions/Exception.h>

#include <string>

#include "rmw/error_handling.h"

#include "type_support_common.hpp"

namespace rmw_fastrtps_cpp
{

uint8_t get_type_support_abi_version(const char * identifier)
{
  uint8_t abi_version = rmw_fastrtps_shared_cpp::TypeSupport::AbiVersion::ABI_V1;

  if (strcmp(identifier, RMW_FASTRTPS_CPP_TYPESUPPORT_C_V2) == 0 ||
      strcmp(identifier, RMW_FASTRTPS_CPP_TYPESUPPORT_CPP_V2) == 0)
  {
    abi_version = rmw_fastrtps_shared_cpp::TypeSupport::AbiVersion::ABI_V2;
  }

  return abi_version;
}

TypeSupport::TypeSupport()
{
  abi_version_ = AbiVersion::ABI_V1;
  m_isGetKeyDefined = false;
  max_size_bound_ = false;
  is_plain_ = false;
  key_is_unbounded_ = false;
  key_max_serialized_size_ = 0;
}

void TypeSupport::set_members(const message_type_support_callbacks_t * members)
{
  members_ = members;

#ifdef ROSIDL_TYPESUPPORT_FASTRTPS_HAS_PLAIN_TYPES
  char bounds_info;
  auto data_size = static_cast<uint32_t>(members->max_serialized_size(bounds_info));
  max_size_bound_ = 0 != (bounds_info & ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE);
  is_plain_ = bounds_info == ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE;
#else
  is_plain_ = true;
  auto data_size = static_cast<uint32_t>(members->max_serialized_size(is_plain_));
  max_size_bound_ = is_plain_;
#endif

  // A plain message of size 0 is an empty message
  if (is_plain_ && (data_size == 0) ) {
    has_data_ = false;
    ++data_size;  // Dummy byte
  } else {
    has_data_ = true;
  }

  // Total size is encapsulation size + data size
  m_typeSize = 4 + data_size;
  // Account for RTPS submessage alignment
  m_typeSize = (m_typeSize + 3) & ~3;
}

void TypeSupport::set_members_v2(const message_type_support_callbacks_t * members)
{

  set_members(members);

  if (nullptr != members->key_callbacks)
  {
    this->key_callbacks_ = members->key_callbacks;
    m_isGetKeyDefined = true;

    this->key_max_serialized_size_ = this->key_callbacks_->max_serialized_key_size(0, this->key_is_unbounded_);
    if (!this->key_is_unbounded_)
    {
      this->key_buffer_.reserve(this->key_max_serialized_size_);
    }
  }
}

size_t TypeSupport::getEstimatedSerializedSize(const void * ros_message, const void * impl) const
{
  if (is_plain_) {
    return m_typeSize;
  }

  assert(ros_message);
  assert(impl);

  auto callbacks = static_cast<const message_type_support_callbacks_t *>(impl);

  // Encapsulation size + message size
  return 4 + callbacks->get_serialized_size(ros_message);
}

bool TypeSupport::serializeROSmessage(
  const void * ros_message, eprosima::fastcdr::Cdr & ser, const void * impl) const
{
  assert(ros_message);
  assert(impl);

  // Serialize encapsulation
  ser.serialize_encapsulation();

  // If type is not empty, serialize message
  if (has_data_) {
    auto callbacks = static_cast<const message_type_support_callbacks_t *>(impl);
    return callbacks->cdr_serialize(ros_message, ser);
  }

  // Otherwise, add a dummy byte
  ser << (uint8_t)0;
  return true;
}

bool TypeSupport::deserializeROSmessage(
  eprosima::fastcdr::Cdr & deser, void * ros_message, const void * impl) const
{
  assert(ros_message);
  assert(impl);

  try {
    // Deserialize encapsulation.
    deser.read_encapsulation();

    // If type is not empty, deserialize message
    if (has_data_) {
      auto callbacks = static_cast<const message_type_support_callbacks_t *>(impl);
      return callbacks->cdr_deserialize(deser, ros_message);
    }

    // Otherwise, consume dummy byte
    uint8_t dump = 0;
    deser >> dump;
    (void)dump;
  } catch (const eprosima::fastcdr::exception::Exception &) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "Fast CDR exception deserializing message of type %s.",
      getName());
    return false;
  } catch (const std::bad_alloc &) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "'Bad alloc' exception deserializing message of type %s.",
      getName());
    return false;
  }

  return true;
}

bool TypeSupport::getKeyHashFromROSmessage(
    void * ros_message,
    eprosima::fastrtps::rtps::InstanceHandle_t * ihandle,
    bool force_md5,
    const void * impl) const
{
  assert(ros_message);
  (void)impl;

  /*if (capacity < 16)
  {
    throw std::runtime_error("Not enough capacity to serialize key");
  }*/

  //! retrieve estimated serialized size in case key is unbounded
  if (this->key_is_unbounded_)
  {
    this->key_max_serialized_size_ = key_callbacks_->get_serialized_key_size(ros_message, 0);
    this->key_buffer_.reserve(this->key_max_serialized_size_);
  }

  eprosima::fastcdr::FastBuffer fast_buffer(
    reinterpret_cast<char *>(this->key_buffer_.data()),
    this->key_max_serialized_size_);

  eprosima::fastcdr::Cdr ser(
    fast_buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);

  key_callbacks_->cdr_serialize_key(ros_message, ser);

    //! check for md5
  if (force_md5 || this->key_max_serialized_size_ > 16)
  {

    this->md5_.init();

#if FASTCDR_VERSION_MAJOR == 1
    this->md5_.update(this->key_buffer_.data(), static_cast<unsigned int>(ser.getSerializedDataLength()));
#else
    this->md5_.update(this->key_buffer_.data(), static_cast<unsigned int>(ser.get_serialized_data_length()));
#endif // FASTCDR_VERSION_MAJOR == 1

    this->md5_.finalize();

    for (uint8_t i = 0; i < 16; ++i)
    {
      ihandle->value[i] = md5_.digest[i];
    }
  }
  else
  {
    for (uint8_t i = 0; i < 16; ++i)
    {
        ihandle->value[i] = this->key_buffer_[i];
    }
  }

  return true;
}

MessageTypeSupport::MessageTypeSupport(
  const message_type_support_callbacks_t * members,
  uint8_t abi_version)
{
  assert(members);

  abi_version_ = abi_version;

  std::string name = _create_type_name(members);
  this->setName(name.c_str());

  switch (abi_version)
  {
    case TypeSupport::AbiVersion::ABI_V1:
    {
      set_members(members);
      break;
    }
    case TypeSupport::AbiVersion::ABI_V2:
    {
      set_members_v2(members);
      break;
    }
    default:
    {
      set_members(members);
      break;
    }
  }
}

ServiceTypeSupport::ServiceTypeSupport()
{
}

RequestTypeSupport::RequestTypeSupport(const service_type_support_callbacks_t * members)
{
  assert(members);

  auto msg = static_cast<const message_type_support_callbacks_t *>(
    members->request_members_->data);
  std::string name = _create_type_name(msg);  // + "Request_";
  this->setName(name.c_str());

  set_members(msg);
}

ResponseTypeSupport::ResponseTypeSupport(const service_type_support_callbacks_t * members)
{
  assert(members);

  auto msg = static_cast<const message_type_support_callbacks_t *>(
    members->response_members_->data);
  std::string name = _create_type_name(msg);  // + "Response_";
  this->setName(name.c_str());

  set_members(msg);
}

}  // namespace rmw_fastrtps_cpp
