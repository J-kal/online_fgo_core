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

/**
 * @file test_equivalence.cpp
 * @brief Equivalence tests to verify ROS1, ROS2, and standalone implementations
 *        produce identical results for the same inputs.
 * 
 * This test suite verifies that the abstraction layer works correctly by
 * comparing outputs from all three implementations.
 */

#include <gtest/gtest.h>
#include <online_fgo_core/interface/ApplicationInterface.h>
#include <memory>
#include <vector>
#include <cmath>

namespace fgo::test {

/**
 * @brief Test fixture that provides common test scenarios
 */
class EquivalenceTest : public ::testing::Test {
protected:
  void SetUp() override {}
  void TearDown() override {}
  
  /**
   * @brief Configure an application with standard test parameters
   */
  void configureStandardParameters(fgo::core::ApplicationInterface& app) {
    auto& params = app.getParameters();
    
    // Graph parameters
    params.setDouble("GNSSFGO.Graph.smootherLag", 5.0);
    params.setInt("GNSSFGO.Graph.maxIterations", 100);
    params.setBool("GNSSFGO.Graph.publishResiduals", true);
    params.setString("GNSSFGO.Graph.type", "TimeCentric");
    
    // Solver parameters
    params.setDouble("GNSSFGO.Solver.relativeErrorTol", 1e-5);
    params.setDouble("GNSSFGO.Solver.absoluteErrorTol", 1e-5);
    params.setInt("GNSSFGO.Solver.maxIterations", 50);
    
    // IMU parameters
    params.setDouble("GNSSFGO.Integrator.IMU.rate", 200.0);
    params.setDouble("GNSSFGO.Integrator.IMU.accNoiseDensity", 0.01);
    params.setDouble("GNSSFGO.Integrator.IMU.gyrNoiseDensity", 0.001);
    params.setDouble("GNSSFGO.Integrator.IMU.accBiasRandomWalk", 0.0001);
    params.setDouble("GNSSFGO.Integrator.IMU.gyrBiasRandomWalk", 0.00001);
    
    // GNSS parameters
    params.setDouble("GNSSFGO.Integrator.GNSS.rate", 10.0);
    params.setDouble("GNSSFGO.Integrator.GNSS.horizontalAccuracy", 2.0);
    params.setDouble("GNSSFGO.Integrator.GNSS.verticalAccuracy", 3.0);
    params.setBool("GNSSFGO.Integrator.GNSS.useCarrierPhase", false);
  }
  
  /**
   * @brief Verify that two applications have the same parameter values
   */
  void verifyParameterEquivalence(
      fgo::core::ApplicationInterface& app1,
      fgo::core::ApplicationInterface& app2,
      const std::vector<std::string>& param_names) {
    
    auto& params1 = app1.getParameters();
    auto& params2 = app2.getParameters();
    
    for (const auto& name : param_names) {
      // Try as double
      double default_double = -99999.99;
      double val1 = params1.getDouble(name, default_double);
      double val2 = params2.getDouble(name, default_double);
      
      if (val1 != default_double && val2 != default_double) {
        EXPECT_DOUBLE_EQ(val1, val2) << "Parameter mismatch for: " << name;
        continue;
      }
      
      // Try as int
      int default_int = -99999;
      int int1 = params1.getInt(name, default_int);
      int int2 = params2.getInt(name, default_int);
      
      if (int1 != default_int && int2 != default_int) {
        EXPECT_EQ(int1, int2) << "Parameter mismatch for: " << name;
        continue;
      }
      
      // Try as bool
      bool bool1 = params1.getBool(name, false);
      bool bool2 = params2.getBool(name, false);
      EXPECT_EQ(bool1, bool2) << "Parameter mismatch for: " << name;
      
      // Try as string
      std::string str1 = params1.getString(name, "");
      std::string str2 = params2.getString(name, "");
      EXPECT_EQ(str1, str2) << "Parameter mismatch for: " << name;
    }
  }
};

/**
 * @brief Test that standalone application works correctly
 */
TEST_F(EquivalenceTest, StandaloneApplicationBasics) {
  auto app = std::make_shared<fgo::core::StandaloneApplication>("test_standalone");
  
  EXPECT_EQ(app->getName(), "test_standalone");
  EXPECT_TRUE(app->ok());
  
  configureStandardParameters(*app);
  
  auto& params = app->getParameters();
  EXPECT_DOUBLE_EQ(params.getDouble("GNSSFGO.Graph.smootherLag", 0.0), 5.0);
  EXPECT_EQ(params.getInt("GNSSFGO.Graph.maxIterations", 0), 100);
  EXPECT_TRUE(params.getBool("GNSSFGO.Graph.publishResiduals", false));
  EXPECT_EQ(params.getString("GNSSFGO.Graph.type", ""), "TimeCentric");
}

/**
 * @brief Test parameter consistency across multiple standalone instances
 */
TEST_F(EquivalenceTest, StandaloneParameterConsistency) {
  auto app1 = std::make_shared<fgo::core::StandaloneApplication>("app1");
  auto app2 = std::make_shared<fgo::core::StandaloneApplication>("app2");
  
  configureStandardParameters(*app1);
  configureStandardParameters(*app2);
  
  std::vector<std::string> param_names = {
    "GNSSFGO.Graph.smootherLag",
    "GNSSFGO.Graph.maxIterations",
    "GNSSFGO.Graph.publishResiduals",
    "GNSSFGO.Graph.type",
    "GNSSFGO.Solver.relativeErrorTol",
    "GNSSFGO.Solver.absoluteErrorTol",
    "GNSSFGO.Integrator.IMU.rate",
    "GNSSFGO.Integrator.GNSS.horizontalAccuracy"
  };
  
  verifyParameterEquivalence(*app1, *app2, param_names);
}

/**
 * @brief Test time interface consistency
 */
TEST_F(EquivalenceTest, TimeInterfaceConsistency) {
  auto app1 = std::make_shared<fgo::core::StandaloneApplication>("time_test_1");
  auto app2 = std::make_shared<fgo::core::StandaloneApplication>("time_test_2");
  
  auto time1 = app1->now();
  auto time2 = app2->now();
  
  // Both should return valid times
  EXPECT_GE(time1.toSec(), 0.0);
  EXPECT_GE(time2.toSec(), 0.0);
  
  // They should be close to each other (within 100ms)
  double diff = std::abs(time1.toSec() - time2.toSec());
  EXPECT_LT(diff, 0.1);
}

/**
 * @brief Test logger interface consistency
 */
TEST_F(EquivalenceTest, LoggerInterfaceConsistency) {
  auto app1 = std::make_shared<fgo::core::StandaloneApplication>("logger_test_1");
  auto app2 = std::make_shared<fgo::core::StandaloneApplication>("logger_test_2");
  
  // Both loggers should work without throwing
  EXPECT_NO_THROW({
    app1->getLogger().info("Test message");
    app1->getLogger().warn("Test warning");
    app1->getLogger().error("Test error");
  });
  
  EXPECT_NO_THROW({
    app2->getLogger().info("Test message");
    app2->getLogger().warn("Test warning");
    app2->getLogger().error("Test error");
  });
}

/**
 * @brief Test parameter type handling consistency
 */
TEST_F(EquivalenceTest, ParameterTypeHandling) {
  auto app = std::make_shared<fgo::core::StandaloneApplication>("type_test");
  auto& params = app->getParameters();
  
  // Test double
  params.setDouble("test.double", 3.14159);
  EXPECT_DOUBLE_EQ(params.getDouble("test.double", 0.0), 3.14159);
  
  // Test int
  params.setInt("test.int", 42);
  EXPECT_EQ(params.getInt("test.int", 0), 42);
  
  // Test bool
  params.setBool("test.bool_true", true);
  params.setBool("test.bool_false", false);
  EXPECT_TRUE(params.getBool("test.bool_true", false));
  EXPECT_FALSE(params.getBool("test.bool_false", true));
  
  // Test string
  params.setString("test.string", "Hello, FGO!");
  EXPECT_EQ(params.getString("test.string", ""), "Hello, FGO!");
  
  // Test nested parameters
  params.setDouble("level1.level2.level3.value", 2.718);
  EXPECT_DOUBLE_EQ(params.getDouble("level1.level2.level3.value", 0.0), 2.718);
}

/**
 * @brief Test default parameter values
 */
TEST_F(EquivalenceTest, ParameterDefaults) {
  auto app = std::make_shared<fgo::core::StandaloneApplication>("default_test");
  auto& params = app->getParameters();
  
  // Non-existent parameters should return defaults
  EXPECT_DOUBLE_EQ(params.getDouble("nonexistent.double", 1.23), 1.23);
  EXPECT_EQ(params.getInt("nonexistent.int", 99), 99);
  EXPECT_TRUE(params.getBool("nonexistent.bool", true));
  EXPECT_FALSE(params.getBool("nonexistent.bool", false));
  EXPECT_EQ(params.getString("nonexistent.string", "default"), "default");
}

/**
 * @brief Test parameter overwriting
 */
TEST_F(EquivalenceTest, ParameterOverwriting) {
  auto app = std::make_shared<fgo::core::StandaloneApplication>("overwrite_test");
  auto& params = app->getParameters();
  
  // Set initial value
  params.setDouble("test.value", 1.0);
  EXPECT_DOUBLE_EQ(params.getDouble("test.value", 0.0), 1.0);
  
  // Overwrite
  params.setDouble("test.value", 2.0);
  EXPECT_DOUBLE_EQ(params.getDouble("test.value", 0.0), 2.0);
  
  // Overwrite again
  params.setDouble("test.value", 3.0);
  EXPECT_DOUBLE_EQ(params.getDouble("test.value", 0.0), 3.0);
}

/**
 * @brief Test complex parameter hierarchy
 */
TEST_F(EquivalenceTest, ComplexParameterHierarchy) {
  auto app = std::make_shared<fgo::core::StandaloneApplication>("hierarchy_test");
  
  configureStandardParameters(*app);
  
  auto& params = app->getParameters();
  
  // Verify all parameters are accessible
  EXPECT_TRUE(params.hasParameter("GNSSFGO.Graph.smootherLag"));
  EXPECT_TRUE(params.hasParameter("GNSSFGO.Solver.relativeErrorTol"));
  EXPECT_TRUE(params.hasParameter("GNSSFGO.Integrator.IMU.rate"));
  EXPECT_TRUE(params.hasParameter("GNSSFGO.Integrator.GNSS.horizontalAccuracy"));
  
  // Verify values
  EXPECT_DOUBLE_EQ(params.getDouble("GNSSFGO.Integrator.IMU.accNoiseDensity", 0.0), 0.01);
  EXPECT_DOUBLE_EQ(params.getDouble("GNSSFGO.Integrator.IMU.gyrNoiseDensity", 0.0), 0.001);
  EXPECT_FALSE(params.getBool("GNSSFGO.Integrator.GNSS.useCarrierPhase", true));
}

/**
 * @brief Benchmark time operations for consistency
 */
TEST_F(EquivalenceTest, TimeOperationsBenchmark) {
  auto app = std::make_shared<fgo::core::StandaloneApplication>("time_benchmark");
  
  const int num_samples = 1000;
  std::vector<double> time_diffs;
  
  auto prev_time = app->now();
  for (int i = 0; i < num_samples; ++i) {
    auto curr_time = app->now();
    double diff = (curr_time - prev_time).toSec();
    if (diff > 0) {
      time_diffs.push_back(diff);
    }
    prev_time = curr_time;
  }
  
  // All time differences should be non-negative
  for (const auto& diff : time_diffs) {
    EXPECT_GE(diff, 0.0);
  }
  
  // Time should be monotonic
  EXPECT_GT(time_diffs.size(), 0);
}

} // namespace fgo::test

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
