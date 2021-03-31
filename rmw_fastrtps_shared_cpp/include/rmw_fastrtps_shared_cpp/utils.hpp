// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef RMW_FASTRTPS_SHARED_CPP__UTILS_HPP_
#define RMW_FASTRTPS_SHARED_CPP__UTILS_HPP_

#include <mutex>
#include <string>

#include "fastdds/dds/topic/TopicDescription.hpp"
#include "fastdds/dds/topic/TypeSupport.hpp"

#include "fastrtps/types/TypesBase.h"

#include "rmw_fastrtps_shared_cpp/custom_participant_info.hpp"
#include "rmw_fastrtps_shared_cpp/TypeSupport.hpp"

#include "rmw/rmw.h"


namespace rmw_fastrtps_shared_cpp
{

/**
* Auxiliary struct to cleanup a topic created during entity creation.
* It is similar to a unique_ptr and its custom deleter at the same time.
*
* The creation process of an entity should be as follows:
* - find_and_check_topic_and_type is called
* - If the type is not found, it is created and registered
* - cast_or_create_topic is called on a stack-allocated TopicHolder
* - An early return will delete the topic if necessary
* - create_datawriter or create_datareader is called
* - Rest of the initialization process is performed
* - Before correctly returning the created entity, field should_be_deleted is set to false
*   to avoid deletion of the topic
*/
struct TopicHolder
{
  ~TopicHolder()
  {
    if (should_be_deleted) {
      participant->delete_topic(topic);
    }
  }

  eprosima::fastdds::dds::DomainParticipant * participant = nullptr;
  eprosima::fastdds::dds::TopicDescription * desc = nullptr;
  eprosima::fastdds::dds::Topic * topic = nullptr;
  bool should_be_deleted = false;
};

/**
* Convert a Fast DDS return code into the corresponding rmw_ret_t
* \param code The Fast DDS return code to be translated
* \return the corresponding rmw_ret_t value
*/
RMW_FASTRTPS_SHARED_CPP_PUBLIC
rmw_ret_t
cast_error_dds_to_rmw(eprosima::fastrtps::types::ReturnCode_t code);

/**
* Auxiliary method to reuse or create a topic during the creation of an entity.
*
* Input:
* \param participant     DomainParticipant where the topic will be created.
* \param desc            TopicDescription returned by find_and_check_topic_and_type.
* \param topic_name      Name of the topic.
* \param type_name       Name of the type.
* \param topic_qos       QoS with which to create the topic.
* \param is_writer_topic Whether the resulting topic will be used on a DataWriter.
*
* Output:
* \param topic_holder    Will hold the pointer to the topic along with the necessary
*                        information for its deletion.
*                        When is_writer_topic is true, topic_holder->topic can be
*                        directly used on a create_datawriter call.
*                        When is_writer_topic is false, topic_holder->desc can be
*                        directly used on a create_datareader call.
*
* \return true when the topic was reused (topic_holder->should_be_deleted will be false)
* \return true when the topic was created (topic_holder->should_be_deleted will be true)
* \return false when the topic could not be created
*/
RMW_FASTRTPS_SHARED_CPP_PUBLIC
bool
cast_or_create_topic(
  eprosima::fastdds::dds::DomainParticipant * participant,
  eprosima::fastdds::dds::TopicDescription * desc,
  const std::string & topic_name,
  const std::string & type_name,
  const eprosima::fastdds::dds::TopicQos & topic_qos,
  bool is_writer_topic,
  TopicHolder * topic_holder);

/**
* Tries to find already registered topic and type.
*
* Input:
* \param participant_info CustomParticipantInfo associated to the context.
* \param topic_name       Name of the topic for the entity being created.
* \param type_name        Name of the type for the entity being created.
*
* Output:
* \param returned_topic   TopicDescription for topic_name
* \param returned_type    TypeSupport for type_name
*
* \return false if `topic_name` was previously created with a different type name.
* \return true when there is no such conflict. Returned topic and type may be null
*              if they were not previously registered on the participant.
*/
RMW_FASTRTPS_SHARED_CPP_PUBLIC
bool
find_and_check_topic_and_type(
  const CustomParticipantInfo * participant_info,
  const std::string & topic_name,
  const std::string & type_name,
  eprosima::fastdds::dds::TopicDescription ** returned_topic,
  eprosima::fastdds::dds::TypeSupport * returned_type);

/**
* Performs removal of associated topic and type.
*
* \param participant_info CustomParticipantInfo associated to the context.
* \param topic            Topic of the entity being deleted.
* \param type             TypeSupport of the entity being deleted.
*/
RMW_FASTRTPS_SHARED_CPP_PUBLIC
void
remove_topic_and_type(
  const CustomParticipantInfo * participant_info,
  const eprosima::fastdds::dds::TopicDescription * topic,
  const eprosima::fastdds::dds::TypeSupport & type);

}  // namespace rmw_fastrtps_shared_cpp

#endif  // RMW_FASTRTPS_SHARED_CPP__UTILS_HPP_
