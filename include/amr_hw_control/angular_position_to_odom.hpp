#ifndef ODOM_PUBLISHER_HPP
#define ODOM_PUBLISHER_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>


class OdomPublisher {
public:
    OdomPublisher();

    void motorResponseCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_motor_response;
    ros::Publisher pub_odom;
    ros::Publisher joint_state_pub; // Declare joint state publisher
    float x, y, theta;
    ros::Time last_time;
    tf::TransformBroadcaster odom_broadcaster;
    tf::TransformBroadcaster tf_broadcaster;

};

#endif // ODOM_PUBLISHER_HPP