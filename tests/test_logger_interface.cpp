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
#include <online_fgo_core/interface/LoggerInterface.h>
#include <vector>
#include <string>

namespace fgo::core::test {

// Mock logger for testing
class MockLogger : public LoggerInterface {
public:
  struct LogEntry {
    enum class Level { INFO, WARN, ERROR, DEBUG };
    Level level;
    std::string message;
  };

  std::vector<LogEntry> logs;
  bool debug_enabled = false;

  void info(const std::string& message) override {
    logs.push_back({LogEntry::Level::INFO, message});
  }

  void warn(const std::string& message) override {
    logs.push_back({LogEntry::Level::WARN, message});
  }

  void error(const std::string& message) override {
    logs.push_back({LogEntry::Level::ERROR, message});
  }

  void debug(const std::string& message) override {
    logs.push_back({LogEntry::Level::DEBUG, message});
  }

  bool isDebugEnabled() const override {
    return debug_enabled;
  }

  void clear() {
    logs.clear();
  }
};

class LoggerInterfaceTest : public ::testing::Test {
protected:
  void SetUp() override {
    logger = std::make_unique<MockLogger>();
  }

  void TearDown() override {
    logger.reset();
  }

  std::unique_ptr<MockLogger> logger;
};

TEST_F(LoggerInterfaceTest, InfoLogging) {
  logger->info("Test info message");
  
  ASSERT_EQ(logger->logs.size(), 1);
  EXPECT_EQ(logger->logs[0].level, MockLogger::LogEntry::Level::INFO);
  EXPECT_EQ(logger->logs[0].message, "Test info message");
}

TEST_F(LoggerInterfaceTest, WarnLogging) {
  logger->warn("Test warning");
  
  ASSERT_EQ(logger->logs.size(), 1);
  EXPECT_EQ(logger->logs[0].level, MockLogger::LogEntry::Level::WARN);
  EXPECT_EQ(logger->logs[0].message, "Test warning");
}

TEST_F(LoggerInterfaceTest, ErrorLogging) {
  logger->error("Test error");
  
  ASSERT_EQ(logger->logs.size(), 1);
  EXPECT_EQ(logger->logs[0].level, MockLogger::LogEntry::Level::ERROR);
  EXPECT_EQ(logger->logs[0].message, "Test error");
}

TEST_F(LoggerInterfaceTest, DebugLogging) {
  logger->debug_enabled = true;
  logger->debug("Test debug");
  
  ASSERT_EQ(logger->logs.size(), 1);
  EXPECT_EQ(logger->logs[0].level, MockLogger::LogEntry::Level::DEBUG);
  EXPECT_EQ(logger->logs[0].message, "Test debug");
}

TEST_F(LoggerInterfaceTest, DebugLoggingDisabled) {
  logger->debug_enabled = false;
  EXPECT_FALSE(logger->isDebugEnabled());
  
  logger->debug("This should be logged");
  ASSERT_EQ(logger->logs.size(), 1);  // Debug is still called
}

TEST_F(LoggerInterfaceTest, StreamStyleInfo) {
  logger->infoStream("Value: ", 42, ", Name: ", "test");
  
  ASSERT_EQ(logger->logs.size(), 1);
  EXPECT_EQ(logger->logs[0].level, MockLogger::LogEntry::Level::INFO);
  EXPECT_EQ(logger->logs[0].message, "Value: 42, Name: test");
}

TEST_F(LoggerInterfaceTest, StreamStyleWarn) {
  logger->warnStream("Warning: ", 3.14, " degrees");
  
  ASSERT_EQ(logger->logs.size(), 1);
  EXPECT_EQ(logger->logs[0].level, MockLogger::LogEntry::Level::WARN);
  EXPECT_EQ(logger->logs[0].message, "Warning: 3.14 degrees");
}

TEST_F(LoggerInterfaceTest, StreamStyleError) {
  logger->errorStream("Error code: ", -1);
  
  ASSERT_EQ(logger->logs.size(), 1);
  EXPECT_EQ(logger->logs[0].level, MockLogger::LogEntry::Level::ERROR);
  EXPECT_EQ(logger->logs[0].message, "Error code: -1");
}

TEST_F(LoggerInterfaceTest, StreamStyleDebugEnabled) {
  logger->debug_enabled = true;
  logger->debugStream("Debug: x=", 10, " y=", 20);
  
  ASSERT_EQ(logger->logs.size(), 1);
  EXPECT_EQ(logger->logs[0].level, MockLogger::LogEntry::Level::DEBUG);
  EXPECT_EQ(logger->logs[0].message, "Debug: x=10 y=20");
}

TEST_F(LoggerInterfaceTest, StreamStyleDebugDisabled) {
  logger->debug_enabled = false;
  logger->debugStream("This should not be in logs");
  
  // With debug disabled, the stream operation is skipped
  EXPECT_EQ(logger->logs.size(), 0);
}

TEST_F(LoggerInterfaceTest, MultipleLogEntries) {
  logger->info("First");
  logger->warn("Second");
  logger->error("Third");
  
  ASSERT_EQ(logger->logs.size(), 3);
  EXPECT_EQ(logger->logs[0].message, "First");
  EXPECT_EQ(logger->logs[1].message, "Second");
  EXPECT_EQ(logger->logs[2].message, "Third");
}

} // namespace fgo::core::test

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
