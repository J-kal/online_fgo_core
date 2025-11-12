//  Copyright 2024 Institute of Automatic Control RWTH Aachen University
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//
//  Author: Haoming Zhang (h.zhang@irt.rwth-aachen.de)
//

#ifndef ONLINE_FGO_CORE_TIME_INTERFACE_H
#define ONLINE_FGO_CORE_TIME_INTERFACE_H

#pragma once

#include <chrono>
#include <cstdint>

namespace fgo::core {

  /**
   * @brief Framework-agnostic timestamp representation
   * 
   * Replaces rclcpp::Time with a portable implementation that can
   * be adapted to ROS1, ROS2, or standalone C++ applications.
   */
  class TimeStamp {
  public:
    /**
     * @brief Construct a timestamp from seconds and nanoseconds
     * @param seconds Whole seconds
     * @param nanoseconds Fractional nanoseconds (0-999999999)
     */
    TimeStamp(int64_t seconds = 0, uint32_t nanoseconds = 0)
      : seconds_(seconds), nanoseconds_(nanoseconds) {
      normalize();
    }

    /**
     * @brief Construct a timestamp from a double (seconds.fractional)
     * @param time_seconds Time in seconds with fractional part
     */
    explicit TimeStamp(double time_seconds) {
      seconds_ = static_cast<int64_t>(time_seconds);
      nanoseconds_ = static_cast<uint32_t>((time_seconds - seconds_) * 1e9);
      normalize();
    }

    /**
     * @brief Get current time using system clock
     * @return Current timestamp
     */
    static TimeStamp now() {
      auto now = std::chrono::system_clock::now();
      auto duration = now.time_since_epoch();
      auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
      auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);
      return TimeStamp(seconds.count(), nanoseconds.count());
    }

    /**
     * @brief Create a zero timestamp
     */
    static TimeStamp zero() {
      return TimeStamp(0, 0);
    }

    /**
     * @brief Convert to seconds (double precision)
     * @return Time in seconds with fractional part
     */
    double toSec() const {
      return static_cast<double>(seconds_) + static_cast<double>(nanoseconds_) * 1e-9;
    }

    /**
     * @brief Get seconds component
     */
    int64_t seconds() const { return seconds_; }

    /**
     * @brief Get nanoseconds component
     */
    uint32_t nanoseconds() const { return nanoseconds_; }

    /**
     * @brief Convert from seconds
     */
    void fromSec(double seconds) {
      seconds_ = static_cast<int64_t>(seconds);
      nanoseconds_ = static_cast<uint32_t>((seconds - seconds_) * 1e9);
      normalize();
    }

    // Comparison operators
    bool operator==(const TimeStamp& other) const {
      return seconds_ == other.seconds_ && nanoseconds_ == other.nanoseconds_;
    }

    bool operator!=(const TimeStamp& other) const {
      return !(*this == other);
    }

    bool operator<(const TimeStamp& other) const {
      if (seconds_ < other.seconds_) return true;
      if (seconds_ > other.seconds_) return false;
      return nanoseconds_ < other.nanoseconds_;
    }

    bool operator<=(const TimeStamp& other) const {
      return *this < other || *this == other;
    }

    bool operator>(const TimeStamp& other) const {
      return !(*this <= other);
    }

    bool operator>=(const TimeStamp& other) const {
      return !(*this < other);
    }

    // Arithmetic operators
    TimeStamp operator+(const TimeStamp& other) const {
      return TimeStamp(seconds_ + other.seconds_, nanoseconds_ + other.nanoseconds_);
    }

    TimeStamp operator-(const TimeStamp& other) const {
      int64_t sec_diff = seconds_ - other.seconds_;
      int32_t nsec_diff = static_cast<int32_t>(nanoseconds_) - static_cast<int32_t>(other.nanoseconds_);
      
      if (nsec_diff < 0) {
        sec_diff -= 1;
        nsec_diff += 1000000000;
      }
      
      return TimeStamp(sec_diff, static_cast<uint32_t>(nsec_diff));
    }

    TimeStamp& operator+=(const TimeStamp& other) {
      seconds_ += other.seconds_;
      nanoseconds_ += other.nanoseconds_;
      normalize();
      return *this;
    }

    TimeStamp& operator-=(const TimeStamp& other) {
      seconds_ -= other.seconds_;
      if (nanoseconds_ < other.nanoseconds_) {
        seconds_ -= 1;
        nanoseconds_ += 1000000000;
      }
      nanoseconds_ -= other.nanoseconds_;
      normalize();
      return *this;
    }

  private:
    int64_t seconds_;
    uint32_t nanoseconds_;

    void normalize() {
      while (nanoseconds_ >= 1000000000) {
        nanoseconds_ -= 1000000000;
        seconds_ += 1;
      }
    }
  };

  /**
   * @brief Duration representation
   */
  using Duration = TimeStamp;

  // Utility functions for compatibility with existing code
  
  /**
   * @brief Convert TimeStamp to seconds (double)
   * @param timestamp The timestamp to convert
   * @return Time in seconds
   */
  inline double toSeconds(const TimeStamp& timestamp) {
    return timestamp.toSec();
  }

  /**
   * @brief Add seconds to a timestamp
   * @param timestamp The base timestamp
   * @param seconds_to_add Seconds to add (can be fractional)
   * @return New timestamp
   */
  inline TimeStamp addSeconds(const TimeStamp& timestamp, double seconds_to_add) {
    return TimeStamp(timestamp.toSec() + seconds_to_add);
  }

} // namespace fgo::core

#endif // ONLINE_FGO_CORE_TIME_INTERFACE_H
