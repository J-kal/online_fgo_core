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

#include <gtest/gtest.h>
#include <online_fgo_core/interface/ParameterInterface.h>
#include <map>
#include <string>

namespace fgo::core::test {

class ParameterInterfaceTest : public ::testing::Test {
protected:
  void SetUp() override {
    params = std::make_unique<MapParameterServer>();
  }

  void TearDown() override {
    params.reset();
  }

  std::unique_ptr<MapParameterServer> params;
};

TEST_F(ParameterInterfaceTest, SetAndGetDouble) {
  params->setDouble("test.value", 3.14159);
  
  EXPECT_TRUE(params->hasParameter("test.value"));
  EXPECT_DOUBLE_EQ(params->getDouble("test.value", 0.0), 3.14159);
}

TEST_F(ParameterInterfaceTest, GetDoubleWithDefault) {
  EXPECT_FALSE(params->hasParameter("nonexistent"));
  EXPECT_DOUBLE_EQ(params->getDouble("nonexistent", 42.0), 42.0);
}

TEST_F(ParameterInterfaceTest, SetAndGetInt) {
  params->setInt("test.count", 42);
  
  EXPECT_TRUE(params->hasParameter("test.count"));
  EXPECT_EQ(params->getInt("test.count", 0), 42);
}

TEST_F(ParameterInterfaceTest, GetIntWithDefault) {
  EXPECT_EQ(params->getInt("nonexistent", -1), -1);
}

TEST_F(ParameterInterfaceTest, SetAndGetBool) {
  params->setBool("test.enabled", true);
  
  EXPECT_TRUE(params->hasParameter("test.enabled"));
  EXPECT_TRUE(params->getBool("test.enabled", false));
}

TEST_F(ParameterInterfaceTest, GetBoolWithDefault) {
  EXPECT_FALSE(params->getBool("nonexistent", false));
  EXPECT_TRUE(params->getBool("nonexistent", true));
}

TEST_F(ParameterInterfaceTest, SetAndGetString) {
  params->setString("test.name", "online_fgo_core");
  
  EXPECT_TRUE(params->hasParameter("test.name"));
  EXPECT_EQ(params->getString("test.name", ""), "online_fgo_core");
}

TEST_F(ParameterInterfaceTest, GetStringWithDefault) {
  EXPECT_EQ(params->getString("nonexistent", "default"), "default");
}

TEST_F(ParameterInterfaceTest, MultipleParameters) {
  params->setDouble("graph.smootherLag", 5.0);
  params->setInt("graph.maxIterations", 100);
  params->setBool("graph.publishResiduals", true);
  params->setString("graph.type", "TimeCentric");
  
  EXPECT_DOUBLE_EQ(params->getDouble("graph.smootherLag", 0.0), 5.0);
  EXPECT_EQ(params->getInt("graph.maxIterations", 0), 100);
  EXPECT_TRUE(params->getBool("graph.publishResiduals", false));
  EXPECT_EQ(params->getString("graph.type", ""), "TimeCentric");
}

TEST_F(ParameterInterfaceTest, ParameterOverwrite) {
  params->setDouble("test.value", 1.0);
  EXPECT_DOUBLE_EQ(params->getDouble("test.value", 0.0), 1.0);
  
  params->setDouble("test.value", 2.0);
  EXPECT_DOUBLE_EQ(params->getDouble("test.value", 0.0), 2.0);
}

TEST_F(ParameterInterfaceTest, NestedParameterNames) {
  params->setDouble("GNSSFGO.Graph.smootherLag", 5.0);
  params->setDouble("GNSSFGO.Solver.maxIterations", 10.0);
  params->setDouble("GNSSFGO.Integrator.IMU.rate", 200.0);
  
  EXPECT_TRUE(params->hasParameter("GNSSFGO.Graph.smootherLag"));
  EXPECT_TRUE(params->hasParameter("GNSSFGO.Solver.maxIterations"));
  EXPECT_TRUE(params->hasParameter("GNSSFGO.Integrator.IMU.rate"));
  
  EXPECT_DOUBLE_EQ(params->getDouble("GNSSFGO.Graph.smootherLag", 0.0), 5.0);
  EXPECT_DOUBLE_EQ(params->getDouble("GNSSFGO.Solver.maxIterations", 0.0), 10.0);
  EXPECT_DOUBLE_EQ(params->getDouble("GNSSFGO.Integrator.IMU.rate", 0.0), 200.0);
}

} // namespace fgo::core::test

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
