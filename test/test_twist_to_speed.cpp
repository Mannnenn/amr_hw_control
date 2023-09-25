#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "amr_hw_control/twist_to_speed.hpp"

class TwistToSpeedTest : public testing::Test {
protected:
    void SetUp() override {
        // Initialize ROS node handle
        ros::NodeHandle nh;

        // Create publishers for testing
        pub_twist = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        pub_right = nh.advertise<std_msgs::Float32>("right_motor/cmd_vel", 1);
        pub_left = nh.advertise<std_msgs::Float32>("left_motor/cmd_vel", 1);

        // Create subscriber for testing
        sub = nh.subscribe("right_motor/cmd_vel", 1, &TwistToSpeedTest::callback, this);
    }

    void TearDown() override {
        // Shutdown ROS node handle
        ros::shutdown();
    }

    void callback(const std_msgs::Float32::ConstPtr& message) {
        received_command = message;
    }

    // ROS publishers and subscriber for testing
    ros::Publisher pub_twist;
    ros::Publisher pub_right;
    ros::Publisher pub_left;
    ros::Subscriber sub;

    // Received command for testing
    std_msgs::Float32::ConstPtr received_command;
};

TEST_F(TwistToSpeedTest, testTwistToSpeed) {
    // Create Twist2Float32 object for testing
    TwistToSpeedTest();

    // Create Twist message for testing
    geometry_msgs::Twist twist;
    twist.linear.x = 0.5;
    twist.angular.z = 1.0;

    // Publish Twist message for testing
    pub_twist.publish(twist);

    // Wait for command to be received
    ros::Rate rate(10);
    while (ros::ok() && received_command == nullptr) {
        ros::spinOnce();
        rate.sleep();
    }

    // Check that the received command is correct
    float expected_rpm = 60 * (twist.linear.x + twist.angular.z * 0.35 / 2) / (0.075 * 2 * 3.14) * 19;
    EXPECT_FLOAT_EQ(expected_rpm, received_command->data);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}