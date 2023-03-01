#include <gtest/gtest.h>
#include <ros/ros.h>

class TargetTest : public ::testing::Test {
 public:
  TargetTest() : spinner(0){};
  ~TargetTest(){};

  ros::NodeHandle* node;
  ros::AsyncSpinner* spinner;

  void SetUp() override {
    ::testing::Test::SetUp();
    this->node = new ros::NodeHandle("~");
    this->spinner = new ros::AsyncSpinner(0);
    this->spinner->start();
  };

  void TearDown() override {
    ros::shutdown();
    delete this->spinner;
    delete this->node;
    ::testing::Test::TearDown();
  }
};

TEST_F(TargetTest, test_ok) { ASSERT_TRUE(false); }

int main(int argc, char** argv) {
  ros::init(argc, argv, "hardware_interface_node");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();  // To show debug output in the
                                                // tests
  }

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
