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

#include "online_fgo_core/interface/ApplicationInterface.h"
#include "online_fgo_core/graph/GraphBase.h"
#include "online_fgo_core/graph/GraphTimeCentric.h"
#include "online_fgo_core/integrator/IntegratorBase.h"

namespace fgo::core {

// Explicit template instantiations for publisher types used in graph classes
// These are needed because GraphBase/GraphTimeCentric use createPublisher<T>
// and the template is defined in the base class but not implemented
// (StandaloneApplication overrides createPublisher directly)

template<>
std::shared_ptr<PublisherInterface<fgo::graph::FactorResiduals>> 
ApplicationInterface::createPublisherImpl<fgo::graph::FactorResiduals>(const std::string& /*topic*/) {
  // Default implementation: return a null publisher
  // Derived classes can override createPublisher() to provide actual functionality
  return std::make_shared<NullPublisher<fgo::graph::FactorResiduals>>();
}

template<>
std::shared_ptr<PublisherInterface<fgo::graph::SensorProcessingReport>> 
ApplicationInterface::createPublisherImpl<fgo::graph::SensorProcessingReport>(const std::string& /*topic*/) {
  // Default implementation: return a null publisher
  // Derived classes can override createPublisher() to provide actual functionality
  return std::make_shared<NullPublisher<fgo::graph::SensorProcessingReport>>();
}

} // namespace fgo::core
