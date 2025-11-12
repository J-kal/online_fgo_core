//  Copyright 2022 Institute of Automatic Control RWTH Aachen University
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
//  ROS-agnostic port for online_fgo_core
//

#ifndef ONLINE_FGO_CORE_BUFFER_H
#define ONLINE_FGO_CORE_BUFFER_H

#pragma once

#include <boost/circular_buffer.hpp>
#include <atomic>
#include <mutex>
#include <vector>
#include <utility>
#include <algorithm>
#include <iostream>
#include "online_fgo_core/interface/TimeInterface.h"

namespace fgo::core
{
  template<typename BufferType>
  class CircularDataBuffer
  {
    std::mutex mutex_;
    using ExecutiveMutexLock = std::unique_lock<std::mutex>;

  public:
    boost::circular_buffer<BufferType> buffer;
    boost::circular_buffer<fgo::core::TimeStamp> time_buffer;
    boost::circular_buffer<double> duration_buffer;

    // Temporary buffer to save the data but don't update buffer,
    // This is for delaying the state initialization in graph
    boost::circular_buffer<BufferType> buffer_tmp;
    boost::circular_buffer<fgo::core::TimeStamp> time_buffer_tmp;
    boost::circular_buffer<double> duration_buffer_tmp;

    CircularDataBuffer() {
      buffer.set_capacity(buffer_size);
      duration_buffer.set_capacity(buffer_size);
      time_buffer.set_capacity(buffer_size);
      buffer_tmp.set_capacity(buffer_size);
      duration_buffer_tmp.set_capacity(buffer_size);
      time_buffer_tmp.set_capacity(buffer_size);
    }

    explicit CircularDataBuffer(uint size) : buffer_size(size) {
      buffer.set_capacity(buffer_size);
      duration_buffer.set_capacity(buffer_size);
      time_buffer.set_capacity(buffer_size);
      buffer_tmp.set_capacity(buffer_size);
      duration_buffer_tmp.set_capacity(buffer_size);
      time_buffer_tmp.set_capacity(buffer_size);
    }

    void update_buffer(BufferType data, const fgo::core::TimeStamp &t) {
      ExecutiveMutexLock lock(mutex_);
      buffer.push_back(data);
      time_buffer.push_back(t);
      counter++;
    }

    void update_buffer(BufferType data, const fgo::core::TimeStamp &t, double duration) {
      ExecutiveMutexLock lock(mutex_);
      buffer.push_back(data);
      time_buffer.push_back(t);
      duration_buffer.push_back(duration);
      counter++;
    }

    void update_buffer_tmp(BufferType data, const fgo::core::TimeStamp &t) {
      ExecutiveMutexLock lock(mutex_);
      buffer_tmp.push_back(data);
      time_buffer_tmp.push_back(t);
      counter++;
    }

    void update_buffer_tmp(BufferType data, const fgo::core::TimeStamp &t, double duration) {
      ExecutiveMutexLock lock(mutex_);
      buffer_tmp.push_back(data);
      time_buffer_tmp.push_back(t);
      duration_buffer_tmp.push_back(duration);
      counter++;
    }

    void update_buffer_from_tmp()
    {
      ExecutiveMutexLock lock(mutex_);
      if(buffer_tmp.empty())
      {
        std::cout << "Warning: temp buffer is empty!" << std::endl;
        return;
      }
      for(size_t i = 0; i < buffer_tmp.size(); i++)
      {
        buffer.push_back(buffer_tmp[i]);
        time_buffer.push_back(time_buffer_tmp[i]);
        if(!duration_buffer_tmp.empty())
          duration_buffer.push_back(duration_buffer_tmp[i]);
      }
      buffer_tmp.clear();
      time_buffer_tmp.clear();
      duration_buffer_tmp.clear();
    }

    BufferType get_last_buffer() {
      ExecutiveMutexLock lock(mutex_);
      //check_temp_buffer();
      return buffer.back();
    }

    BufferType get_buffer(const fgo::core::TimeStamp &t) {
      ExecutiveMutexLock lock(mutex_);
      //check_temp_buffer();

      if(buffer.empty())
      {
        BufferType T;
        return T;
      }

      std::vector<std::pair<double, BufferType>> buffer_map;

      for (uint i = 0; i < buffer_size; i++) {
        buffer_map.template emplace_back(std::make_pair(abs(t.seconds() - time_buffer[i].seconds()), buffer[i]));

      }
      std::sort(buffer_map.begin(), buffer_map.end(), [](std::pair<double, BufferType> &a,
                                                        std::pair<double, BufferType> &b) {
          return a.first > b.first;
      });
      return buffer_map.back().second;
    }

    BufferType get_buffer(const double &t) {
        ExecutiveMutexLock lock(mutex_);
        //check_temp_buffer();

        if(buffer.empty())
        {
          BufferType T;
          return T;
        }

        std::vector<std::pair<double, BufferType>> buffer_map;

        for (uint i = 0; i < buffer_size; i++) {
          buffer_map.template emplace_back(std::make_pair(abs(t - time_buffer[i].seconds()), buffer[i]));

        }
        std::sort(buffer_map.begin(), buffer_map.end(), [](std::pair<double, BufferType> &a,
                                                           std::pair<double, BufferType> &b) {
            return a.first > b.first;
        });
        return buffer_map.back().second;
      }

    BufferType get_buffer_from_id(uint id) {
      ExecutiveMutexLock lock(mutex_);
      if (id > buffer.size()) {
        std::cout << "Wrong Buffer ID: " << id << " last data is returned!" << std::endl;
        return buffer.back();
      }
      try
      {
        return buffer.at(id);
      }
      catch(std::exception& ex)
      {
        std::cout << "INVALID ID: " << id << " last data is returned!" << std::endl;
        return buffer.back();
      }
    }

    std::vector<std::pair<fgo::core::TimeStamp, BufferType>> get_all_time_buffer_pair()
    {
      std::vector<std::pair<fgo::core::TimeStamp, BufferType>> pairs;
      ExecutiveMutexLock lock(mutex_);
      for(size_t i = 0; i < buffer.size(); i++)
      {
        pairs.template emplace_back(std::make_pair(time_buffer[i], buffer[i]));
      }
      return pairs;
    }

      std::vector<std::pair<fgo::core::TimeStamp, BufferType>> get_all_time_buffer_pair_and_clean()
      {
        std::vector<std::pair<fgo::core::TimeStamp, BufferType>> pairs;
        ExecutiveMutexLock lock(mutex_);
        for(size_t i = 0; i < buffer.size(); i++)
        {
          pairs.template emplace_back(std::make_pair(time_buffer[i], buffer[i]));
        }
        clean_();
        return pairs;
      }

    std::vector<BufferType> get_all_buffer() {
      std::vector<BufferType> buffers;
      ExecutiveMutexLock lock(mutex_);
      //check_temp_buffer();
      for (const auto &b: buffer) {
        buffers.template emplace_back(b);
      }
      return buffers;
    }

    std::vector<BufferType> get_all_buffer_and_clean() {
      std::vector<BufferType> buffers;
      ExecutiveMutexLock lock(mutex_);
      //check_temp_buffer();
      for (const auto &b: buffer) {
        buffers.template emplace_back(b);
      }
      clean_();
      return buffers;
    }

    std::atomic<uint64_t> counter = 0;
    uint buffer_size = 3;

    // Add size() method for compatibility
    size_t size() const {
      return buffer.size();
    }

    // Add resize_buffer() method for compatibility
    void resize_buffer(size_t new_size) {
      buffer_size = new_size;
      buffer.set_capacity(new_size);
      duration_buffer.set_capacity(new_size);
      time_buffer.set_capacity(new_size);
      buffer_tmp.set_capacity(new_size);
      duration_buffer_tmp.set_capacity(new_size);
      time_buffer_tmp.set_capacity(new_size);
    }

    // Add cleanBeforeTime() method for compatibility
    void cleanBeforeTime(double timestamp) {
      ExecutiveMutexLock lock(mutex_);
      // Remove all elements with timestamp < given timestamp
      while (!time_buffer.empty() && time_buffer.front().toSec() < timestamp) {
        buffer.pop_front();
        time_buffer.pop_front();
        if (!duration_buffer.empty()) {
          duration_buffer.pop_front();
        }
      }
    }

  private:
    void clean_() {
      buffer.clear();
      time_buffer.clear();
      duration_buffer.clear();
      buffer_tmp.clear();
      time_buffer_tmp.clear();
      duration_buffer_tmp.clear();
    }
  };
}

#endif //ONLINE_FGO_CORE_BUFFER_H
