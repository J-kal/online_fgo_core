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

#include "online_fgo_core/interface/ParameterInterface.h"
#include <fstream>
#include <stdexcept>
#include <functional>

// Optional: Include yaml-cpp if available
#ifdef HAVE_YAML_CPP
#include <yaml-cpp/yaml.h>
#endif

namespace fgo::core {

  void MapParameterServer::loadFromYAML(const std::string& filename) {
#ifdef HAVE_YAML_CPP
    try {
      YAML::Node config = YAML::LoadFile(filename);
      
      // Recursively parse YAML and populate parameter maps
      // This is a simplified implementation - you may want to add more sophisticated parsing
      std::function<void(const YAML::Node&, const std::string&)> parseNode;
      parseNode = [this, &parseNode](const YAML::Node& node, const std::string& prefix) {
        for (auto it = node.begin(); it != node.end(); ++it) {
          std::string key = prefix.empty() ? it->first.as<std::string>() 
                                            : prefix + "." + it->first.as<std::string>();
          
          if (it->second.IsScalar()) {
            try {
              // Try to parse as different types
              if (it->second.Tag() == "!!bool" || 
                  it->second.as<std::string>() == "true" || 
                  it->second.as<std::string>() == "false") {
                setBool(key, it->second.as<bool>());
              } else {
                try {
                  setInt(key, it->second.as<int>());
                } catch (...) {
                  try {
                    setDouble(key, it->second.as<double>());
                  } catch (...) {
                    setString(key, it->second.as<std::string>());
                  }
                }
              }
            } catch (const std::exception& e) {
              setString(key, it->second.as<std::string>());
            }
          } else if (it->second.IsSequence()) {
            // Try to parse as array
            try {
              std::vector<double> vec = it->second.as<std::vector<double>>();
              setDoubleArray(key, vec);
            } catch (...) {
              try {
                std::vector<int> vec = it->second.as<std::vector<int>>();
                setIntArray(key, vec);
              } catch (...) {
                std::vector<std::string> vec = it->second.as<std::vector<std::string>>();
                setStringArray(key, vec);
              }
            }
          } else if (it->second.IsMap()) {
            parseNode(it->second, key);
          }
        }
      };
      
      parseNode(config, "");
      
    } catch (const std::exception& e) {
      throw std::runtime_error("Failed to load YAML file: " + std::string(e.what()));
    }
#else
    throw std::runtime_error("YAML support not compiled. Please build with yaml-cpp.");
#endif
  }

} // namespace fgo::core
