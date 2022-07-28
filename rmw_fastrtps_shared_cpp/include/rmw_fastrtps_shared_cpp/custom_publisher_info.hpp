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

#ifndef RMW_FASTRTPS_SHARED_CPP__CUSTOM_PUBLISHER_INFO_HPP_
#define RMW_FASTRTPS_SHARED_CPP__CUSTOM_PUBLISHER_INFO_HPP_

#include <atomic>
#include <condition_variable>
#include <map>
#include <mutex>
#include <set>

#include "fastdds/dds/core/status/BaseStatus.hpp"
#include "fastdds/dds/core/status/DeadlineMissedStatus.hpp"
#include "fastdds/dds/core/status/PublicationMatchedStatus.hpp"
#include "fastdds/dds/publisher/DataWriter.hpp"
#include "fastdds/dds/publisher/DataWriterListener.hpp"
#include "fastdds/dds/topic/Topic.hpp"
#include "fastdds/dds/topic/TypeSupport.hpp"

#include "fastdds/rtps/common/Guid.h"
#include "fastdds/rtps/common/InstanceHandle.h"

#include "rcpputils/thread_safety_annotations.hpp"
#include "rmw/rmw.h"

#include "rmw_fastrtps_shared_cpp/custom_event_info.hpp"


class PubListener;

typedef struct CustomPublisherInfo : public CustomEventInfo
{
  virtual ~CustomPublisherInfo() = default;

  eprosima::fastdds::dds::DataWriter * data_writer_{nullptr};
  PubListener * listener_{nullptr};
  eprosima::fastdds::dds::TypeSupport type_support_;
  const void * type_support_impl_{nullptr};
  rmw_gid_t publisher_gid{};
  const char * typesupport_identifier_{nullptr};

  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  EventListenerInterface *
  getListener() const final;
} CustomPublisherInfo;

class PubListener : public EventListenerInterface, public eprosima::fastdds::dds::DataWriterListener
{
public:
  explicit PubListener(CustomPublisherInfo * info)
  : deadline_changes_(false),
    liveliness_changes_(false),
    conditionVariableList_()
  {
    (void) info;
  }

  // DataWriterListener implementation
  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  void
  on_publication_matched(
    eprosima::fastdds::dds::DataWriter * /* writer */,
    const eprosima::fastdds::dds::PublicationMatchedStatus & info) final
  {
    std::lock_guard<std::mutex> lock(internalMutex_);
    if (info.current_count_change == 1) {
      subscriptions_.insert(eprosima::fastrtps::rtps::iHandle2GUID(info.last_subscription_handle));
    } else if (info.current_count_change == -1) {
      subscriptions_.erase(eprosima::fastrtps::rtps::iHandle2GUID(info.last_subscription_handle));
    }
  }

  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  void
  on_offered_deadline_missed(
    eprosima::fastdds::dds::DataWriter * writer,
    const eprosima::fastdds::dds::OfferedDeadlineMissedStatus & status) final;

  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  void
  on_liveliness_lost(
    eprosima::fastdds::dds::DataWriter * writer,
    const eprosima::fastdds::dds::LivelinessLostStatus & status) final;


  // EventListenerInterface implementation
  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  bool
  hasEvent(rmw_event_type_t event_type) const final;

  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  bool
  takeNextEvent(rmw_event_type_t event_type, void * event_info) final;

  // PubListener API
  size_t subscriptionCount()
  {
    std::lock_guard<std::mutex> lock(internalMutex_);
    return subscriptions_.size();
  }

  void
  attachCondition(std::mutex * conditionMutex, std::condition_variable * conditionVariable)
  {
    std::lock_guard<std::mutex> lock(internalMutex_);
    conditionVariableList_.insert(std::make_pair(conditionVariable, conditionMutex));
    mutexList_.emplace_back(conditionMutex);
    std::sort(mutexList_.begin(), mutexList_.end());
  }

  void
  detachCondition(std::condition_variable * conditionVariable)
  {
    std::lock_guard<std::mutex> lock(internalMutex_);
    std::map<std::condition_variable *, std::mutex *>::iterator it =
      std::find_if(conditionVariableList_.begin(), conditionVariableList_.end(),
      [=](std::pair<std::condition_variable *, std::mutex *> in)
        {
          return conditionVariable == in.first;
        }
      );
    if (it != conditionVariableList_.end())
    {
      mutexList_.erase(std::find(mutexList_.begin(), mutexList_.end(),it->second));
      conditionVariableList_.erase(it);  
    }
  }

private:
  mutable std::mutex internalMutex_;

  std::set<eprosima::fastrtps::rtps::GUID_t> subscriptions_
    RCPPUTILS_TSA_GUARDED_BY(internalMutex_);

  std::atomic_bool deadline_changes_;
  eprosima::fastdds::dds::OfferedDeadlineMissedStatus offered_deadline_missed_status_
    RCPPUTILS_TSA_GUARDED_BY(internalMutex_);

  std::atomic_bool liveliness_changes_;
  eprosima::fastdds::dds::LivelinessLostStatus liveliness_lost_status_
    RCPPUTILS_TSA_GUARDED_BY(internalMutex_);

  std::map<std::condition_variable *, std::mutex *> conditionVariableList_ RCPPUTILS_TSA_GUARDED_BY(
    internalMutex_);
  std::vector<std::mutex *> mutexList_;
};

#endif  // RMW_FASTRTPS_SHARED_CPP__CUSTOM_PUBLISHER_INFO_HPP_
