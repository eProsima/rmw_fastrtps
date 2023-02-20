// Copyright 2023 iRobot Corporation. All Rights Reserved.

#pragma once

#include "rcutils/logging_macros.h"

#include <sys/resource.h>
#include <sys/types.h>

#include <chrono>
#include <cstdint>
#include <thread>

namespace rmw_fastrtps_cpp {

/**
 * @brief Trivial class to measure memory allocations
 */
class MemoryMonitor
{
public:
  MemoryMonitor() = default;

  /**
   * @brief Computes the amount of RSS memory allocated since the last call of this function.
   */
  uint64_t get_memory_delta()
  {
    uint64_t current_usage = get_memory_usage();
    uint64_t delta = current_usage - m_rss_kb;
    m_rss_kb = current_usage;
    return delta;
  }

  /**
   * @brief Computes the current RSS memory usage of this process.
   */
  uint64_t get_memory_usage()
  {
    // Sleep for a little bit to make sure that proc pages are updated
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Get rss
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);

    return usage.ru_maxrss;
  }

  static void log_memory_delta(const char* msg)
  {
    uint64_t delta = get_instance().get_memory_delta();
    RCUTILS_LOG_INFO("%s --> %lu", msg, delta);
  }

  static MemoryMonitor& get_instance()
  {
      static MemoryMonitor the_instance;
      return the_instance;
  }

private:
  uint64_t m_rss_kb {0};
};

}  // namespace rmw_fastrtps_cpp
