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
#include <online_fgo_core/interface/ApplicationInterface.h>
#include <memory>
#include <fstream>

namespace fgo::core::test {

class ApplicationInterfaceTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create standalone application for testing
    app = std::make_shared<fgo::core::StandaloneApplication>("test_app");
  }

  void TearDown() override {
    app.reset();
  }

  std::shared_ptr<ApplicationInterface> app;
};

TEST_F(ApplicationInterfaceTest, GetName) {
  EXPECT_EQ(app->getName(), "test_app");
}

TEST_F(ApplicationInterfaceTest, ApplicationIsOk) {
  EXPECT_TRUE(app->ok());
}

TEST_F(ApplicationInterfaceTest, GetLogger) {
  auto& logger = app->getLogger();
  
  // Should not throw
  EXPECT_NO_THROW(logger.info("Test message"));
  EXPECT_NO_THROW(logger.warn("Test warning"));
  EXPECT_NO_THROW(logger.error("Test error"));
  EXPECT_NO_THROW(logger.debug("Test debug"));
}

TEST_F(ApplicationInterfaceTest, GetParameters) {
  auto& params = app->getParameters();
  
  // Set and get parameters
  params.setDouble("test.value", 42.0);
  EXPECT_DOUBLE_EQ(params.getDouble("test.value", 0.0), 42.0);
  
  params.setInt("test.count", 100);
  EXPECT_EQ(params.getInt("test.count", 0), 100);
  
  params.setBool("test.enabled", true);
  EXPECT_TRUE(params.getBool("test.enabled", false));
  
  params.setString("test.name", "online_fgo");
  EXPECT_EQ(params.getString("test.name", ""), "online_fgo");
}

TEST_F(ApplicationInterfaceTest, GetCurrentTime) {
  auto time1 = app->now();
  auto time2 = app->now();
  
  // Time should be monotonically increasing
  EXPECT_TRUE(time2 >= time1);
}

TEST_F(ApplicationInterfaceTest, ParameterPersistence) {
  auto& params = app->getParameters();
  
  // Set multiple parameters
  params.setDouble("graph.smootherLag", 5.0);
  params.setInt("graph.maxIterations", 100);
  params.setBool("graph.useOptimization", true);
  
  // Values should persist
  EXPECT_DOUBLE_EQ(params.getDouble("graph.smootherLag", 0.0), 5.0);
  EXPECT_EQ(params.getInt("graph.maxIterations", 0), 100);
  EXPECT_TRUE(params.getBool("graph.useOptimization", false));
}

TEST_F(ApplicationInterfaceTest, MultipleApplicationInstances) {
  auto app1 = std::make_shared<fgo::core::StandaloneApplication>("app1");
  auto app2 = std::make_shared<fgo::core::StandaloneApplication>("app2");
  
  EXPECT_EQ(app1->getName(), "app1");
  EXPECT_EQ(app2->getName(), "app2");
  
  // Parameters should be independent
  app1->getParameters().setDouble("value", 1.0);
  app2->getParameters().setDouble("value", 2.0);
  
  EXPECT_DOUBLE_EQ(app1->getParameters().getDouble("value", 0.0), 1.0);
  EXPECT_DOUBLE_EQ(app2->getParameters().getDouble("value", 0.0), 2.0);
}

TEST_F(ApplicationInterfaceTest, LoggerStreamInterface) {
  auto& logger = app->getLogger();
  
  // Test stream-style logging
  EXPECT_NO_THROW(logger.infoStream("Value: ", 42));
  EXPECT_NO_THROW(logger.warnStream("Warning: ", 3.14));
  EXPECT_NO_THROW(logger.errorStream("Error code: ", -1));
  EXPECT_NO_THROW(logger.debugStream("Debug: ", true));
}

#ifdef HAVE_YAML_CPP
TEST_F(ApplicationInterfaceTest, LoadParametersFromYAML) {
  // This test requires yaml-cpp
  // Create a temporary YAML file for testing
  const std::string yaml_content = R"(
GNSSFGO:
  Graph:
    smootherLag: 5.0
    maxIterations: 100
    useOptimization: true
    robotName: "test_robot"
  )";
  
  // Write to temp file
  std::string temp_file = "/tmp/test_params.yaml";
  std::ofstream ofs(temp_file);
  ofs << yaml_content;
  ofs.close();
  
  // Load parameters
  EXPECT_NO_THROW(app->getParameters().loadFromYAML(temp_file));
  
  auto& params = app->getParameters();
  EXPECT_DOUBLE_EQ(params.getDouble("GNSSFGO.Graph.smootherLag", 0.0), 5.0);
  EXPECT_EQ(params.getInt("GNSSFGO.Graph.maxIterations", 0), 100);
  EXPECT_TRUE(params.getBool("GNSSFGO.Graph.useOptimization", false));
  EXPECT_EQ(params.getString("GNSSFGO.Graph.robotName", ""), "test_robot");
  
  // Clean up
  std::remove(temp_file.c_str());
}
#endif

} // namespace fgo::core::test

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
