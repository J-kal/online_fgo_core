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
#include <online_fgo_core/interface/TimeInterface.h>

namespace fgo::core::test {

class TimeInterfaceTest : public ::testing::Test {
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(TimeInterfaceTest, DefaultConstruction) {
  TimeStamp ts;
  EXPECT_EQ(ts.seconds(), 0);
  EXPECT_EQ(ts.nanoseconds(), 0);
  EXPECT_DOUBLE_EQ(ts.toSec(), 0.0);
}

TEST_F(TimeInterfaceTest, ConstructionWithSecondsAndNanoseconds) {
  TimeStamp ts(10, 500000000);  // 10.5 seconds
  
  EXPECT_EQ(ts.seconds(), 10);
  EXPECT_EQ(ts.nanoseconds(), 500000000);
  EXPECT_DOUBLE_EQ(ts.toSec(), 10.5);
}

TEST_F(TimeInterfaceTest, ConstructionFromDouble) {
  TimeStamp ts(12.345);
  
  EXPECT_EQ(ts.seconds(), 12);
  EXPECT_EQ(ts.nanoseconds(), 345000000);
  EXPECT_NEAR(ts.toSec(), 12.345, 1e-9);
}

TEST_F(TimeInterfaceTest, ConversionToDouble) {
  TimeStamp ts1(5, 250000000);  // 5.25 seconds
  EXPECT_DOUBLE_EQ(ts1.toSec(), 5.25);
  
  TimeStamp ts2(0, 123456789);  // 0.123456789 seconds
  EXPECT_NEAR(ts2.toSec(), 0.123456789, 1e-9);
  
  TimeStamp ts3(100, 0);  // 100.0 seconds
  EXPECT_DOUBLE_EQ(ts3.toSec(), 100.0);
}

TEST_F(TimeInterfaceTest, ComparisonOperators) {
  TimeStamp ts1(10, 500000000);
  TimeStamp ts2(10, 500000000);
  TimeStamp ts3(11, 0);
  TimeStamp ts4(10, 600000000);
  
  // Equality
  EXPECT_TRUE(ts1 == ts2);
  EXPECT_FALSE(ts1 == ts3);
  
  // Inequality
  EXPECT_FALSE(ts1 != ts2);
  EXPECT_TRUE(ts1 != ts3);
  
  // Less than
  EXPECT_TRUE(ts1 < ts3);
  EXPECT_TRUE(ts1 < ts4);
  EXPECT_FALSE(ts3 < ts1);
  EXPECT_FALSE(ts1 < ts2);
  
  // Less than or equal
  EXPECT_TRUE(ts1 <= ts2);
  EXPECT_TRUE(ts1 <= ts3);
  EXPECT_FALSE(ts3 <= ts1);
  
  // Greater than
  EXPECT_TRUE(ts3 > ts1);
  EXPECT_TRUE(ts4 > ts1);
  EXPECT_FALSE(ts1 > ts3);
  EXPECT_FALSE(ts1 > ts2);
  
  // Greater than or equal
  EXPECT_TRUE(ts1 >= ts2);
  EXPECT_TRUE(ts3 >= ts1);
  EXPECT_FALSE(ts1 >= ts3);
}

TEST_F(TimeInterfaceTest, ArithmeticOperations) {
  TimeStamp ts1(10, 500000000);  // 10.5 seconds
  TimeStamp ts2(5, 250000000);   // 5.25 seconds
  
  // Addition
  TimeStamp sum = ts1 + ts2;
  EXPECT_EQ(sum.seconds(), 15);
  EXPECT_EQ(sum.nanoseconds(), 750000000);
  EXPECT_DOUBLE_EQ(sum.toSec(), 15.75);
  
  // Subtraction
  TimeStamp diff = ts1 - ts2;
  EXPECT_EQ(diff.seconds(), 5);
  EXPECT_EQ(diff.nanoseconds(), 250000000);
  EXPECT_DOUBLE_EQ(diff.toSec(), 5.25);
}

TEST_F(TimeInterfaceTest, NanosecondOverflow) {
  TimeStamp ts1(10, 900000000);  // 10.9 seconds
  TimeStamp ts2(0, 200000000);   // 0.2 seconds
  
  TimeStamp sum = ts1 + ts2;
  // Should be 11.1 seconds = 11 seconds + 100000000 nanoseconds
  EXPECT_EQ(sum.seconds(), 11);
  EXPECT_EQ(sum.nanoseconds(), 100000000);
  EXPECT_NEAR(sum.toSec(), 11.1, 1e-9);
}

TEST_F(TimeInterfaceTest, NanosecondUnderflow) {
  TimeStamp ts1(10, 100000000);  // 10.1 seconds
  TimeStamp ts2(5, 200000000);   // 5.2 seconds
  
  TimeStamp diff = ts1 - ts2;
  // Should be 4.9 seconds = 4 seconds + 900000000 nanoseconds
  EXPECT_EQ(diff.seconds(), 4);
  EXPECT_EQ(diff.nanoseconds(), 900000000);
  EXPECT_NEAR(diff.toSec(), 4.9, 1e-9);
}

TEST_F(TimeInterfaceTest, ZeroTime) {
  TimeStamp zero;
  TimeStamp ts(5, 0);
  
  EXPECT_TRUE(zero == TimeStamp::zero());
  EXPECT_FALSE(ts == TimeStamp::zero());
  
  TimeStamp result = ts - ts;
  EXPECT_TRUE(result == TimeStamp::zero());
}

TEST_F(TimeInterfaceTest, CopyAndAssignment) {
  TimeStamp ts1(10, 500000000);
  
  // Copy constructor
  TimeStamp ts2(ts1);
  EXPECT_EQ(ts1, ts2);
  
  // Assignment operator
  TimeStamp ts3;
  ts3 = ts1;
  EXPECT_EQ(ts1, ts3);
}

TEST_F(TimeInterfaceTest, DurationCalculation) {
  TimeStamp start(100, 250000000);  // 100.25 seconds
  TimeStamp end(105, 750000000);    // 105.75 seconds
  
  Duration duration = end - start;
  EXPECT_DOUBLE_EQ(duration.toSec(), 5.5);
}

TEST_F(TimeInterfaceTest, HighPrecisionTiming) {
  // Test nanosecond precision
  TimeStamp ts1(0, 1);        // 1 nanosecond
  TimeStamp ts2(0, 2);        // 2 nanoseconds
  
  EXPECT_TRUE(ts2 > ts1);
  
  Duration diff = ts2 - ts1;
  EXPECT_EQ(diff.nanoseconds(), 1);
}

} // namespace fgo::core::test

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
