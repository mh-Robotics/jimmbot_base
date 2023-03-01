/**
 * @file can_msg_wrapper_test.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-06-20
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "jimmbot_base/can_msg_wrapper.hpp"

#include <gtest/gtest.h>

namespace jimmbot_base {
TEST(TargetTest, test_ok) { ASSERT_TRUE(false); }
}  // end namespace jimmbot_base

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}