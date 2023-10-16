#ifndef TWIST_TO_SPEED_HPP
#define TWIST_TO_SPEED_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>

class Twist2Float32 {
public:
    Twist2Float32();
    void callback(const geometry_msgs::Twist::ConstPtr& message);
    std::tuple<float, float> twist2vel(const geometry_msgs::Twist& received_data);

private:
    std_msgs::Float32MultiArray motor_command;
    geometry_msgs::Twist::ConstPtr received_twist;
    ros::Subscriber sub;
    ros::Publisher pub_motor_command;
};

#endif // TWIST_TO_SPEED_HPP