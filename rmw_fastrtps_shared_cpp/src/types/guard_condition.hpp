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

#ifndef TYPES__GUARD_CONDITION_HPP_
#define TYPES__GUARD_CONDITION_HPP_

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <condition_variable>
#include <mutex>
#include <map>
#include <utility>
#include <vector>

#include "rcpputils/thread_safety_annotations.hpp"

class GuardCondition
{
public:
  GuardCondition()
  : hasTriggered_(false),
    conditionVariableList_() {}

  void
  trigger()
  {
    std::lock_guard<std::mutex> lock(internalMutex_);

    if (!conditionVariableList_.empty()) {
      for (auto && c: conditionVariableList_) {
        std::unique_lock<std::mutex> clock(*c.second);
        // the change to hasTriggered_ needs to be mutually exclusive with
        // rmw_wait() which checks hasTriggered() and decides if wait() needs to
        // be called
        hasTriggered_ = true;
        clock.unlock();
        c.first->notify_all();
      }
    } else {
      hasTriggered_ = true;
    }
  }

  void
  attachCondition(std::mutex * conditionMutex, std::condition_variable * conditionVariable)
  {
    std::lock_guard<std::mutex> lock(internalMutex_);
    conditionVariableList_.insert(std::make_pair(conditionVariable, conditionMutex));
  }

  void
  detachCondition(std::condition_variable * conditionVariable)
  {
    std::lock_guard<std::mutex> lock(internalMutex_);
    conditionVariableList_.erase(conditionVariableList_.find(conditionVariable)); 
  }

  bool
  hasTriggered()
  {
    return hasTriggered_;
  }

  bool
  getHasTriggered()
  {
    return hasTriggered_.exchange(false);
  }

private:
  std::mutex internalMutex_;
  std::atomic_bool hasTriggered_;
  std::map<std::condition_variable *, std::mutex *> conditionVariableList_ RCPPUTILS_TSA_GUARDED_BY(
    internalMutex_);

};

#endif  // TYPES__GUARD_CONDITION_HPP_
