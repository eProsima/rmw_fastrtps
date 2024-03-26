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

#include "TypeSupport.hpp"

namespace rmw_fastrtps_dynamic_cpp
{

TypeSupportProxy::TypeSupportProxy(rmw_fastrtps_shared_cpp::TypeSupport * inner_type)
: rmw_fastrtps_shared_cpp::TypeSupport(inner_type->ros_message_type_supports())
{
  set_name(inner_type->get_name());
  max_serialized_type_size = inner_type->max_serialized_type_size;
  is_plain_ = inner_type->is_plain(eprosima::fastdds::dds::XCDR_DATA_REPRESENTATION);
  max_size_bound_ = inner_type->is_bounded();
  is_compute_key_provided = inner_type->is_compute_key_provided;
  key_is_unbounded_ = inner_type->is_key_unbounded();
}

size_t TypeSupportProxy::getEstimatedSerializedSize(
  const void * ros_message, const void * impl) const
{
  auto type_impl = static_cast<const rmw_fastrtps_shared_cpp::TypeSupport *>(impl);
  return type_impl->getEstimatedSerializedSize(ros_message, impl);
}

bool TypeSupportProxy::serializeROSmessage(
  const void * ros_message, eprosima::fastcdr::Cdr & ser, const void * impl) const
{
  auto type_impl = static_cast<const rmw_fastrtps_shared_cpp::TypeSupport *>(impl);
  return type_impl->serializeROSmessage(ros_message, ser, impl);
}

bool TypeSupportProxy::deserializeROSmessage(
  eprosima::fastcdr::Cdr & deser, void * ros_message, const void * impl) const
{
  auto type_impl = static_cast<const rmw_fastrtps_shared_cpp::TypeSupport *>(impl);
  return type_impl->deserializeROSmessage(deser, ros_message, impl);
}

bool TypeSupportProxy::get_key_hash_from_ros_message(
    void * ros_message,
    eprosima::fastdds::rtps::InstanceHandle_t * ihandle,
    bool force_md5,
    const void * impl) const
{
  auto type_impl = static_cast<const rmw_fastrtps_shared_cpp::TypeSupport *>(impl);
  return type_impl->get_key_hash_from_ros_message(ros_message, ihandle, force_md5, impl);
}

}  // namespace rmw_fastrtps_dynamic_cpp
