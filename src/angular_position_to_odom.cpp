#include "amr_hw_control/angular_position_to_odom.hpp"

OdomPublisher::OdomPublisher() {
    // Initialize variables
    x = 0.0;
    y = 0.0;
    theta = 0.0;
    last_time = ros::Time::now();

    left_pos = 0.0;
    right_pos = 0.0;

    // Subscribe to motor response topic
    sub_motor_response = nh.subscribe("motor_response", 1, &OdomPublisher::motorResponseCallback, this);

    // Advertise odom topic
    pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 10);
    joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

}

void OdomPublisher::motorResponseCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    // Get wheel_size and wheel_length from rosparam
    float wheel_size;
    float axle_length;

    ros::NodeHandle nh;
    nh.param<float>("wheel_size", wheel_size, 0.286);
    nh.param<float>("axle_length", axle_length, 0.39);

    // Get motor angular velocity
    float right_vel = msg->data[0];
    float left_vel = msg->data[1];

    // Calculate linear and angular velocity
    float linear_velocity = (left_vel + right_vel) * (wheel_size / 2) / (2.0);
    float angular_velocity = (right_vel - left_vel) * (wheel_size / 2) / (axle_length);

    // Calculate time elapsed since last update
    ros::Time current_time = ros::Time::now();
    float dt = (current_time - last_time).toSec();
    last_time = current_time;

    // Integrate linear and angular velocity to get position and orientation
    x += linear_velocity * cos(theta) * dt;
    y += linear_velocity * sin(theta) * dt;
    theta += angular_velocity * dt;

    // Publish odom topic
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    pub_odom.publish(odom);

    //tf odom->base_link
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);

    // Publish joint state message
    left_pos -= left_vel * dt;
    right_pos -= right_vel * dt;
    left_pos = angles::normalize_angle(left_pos);
    right_pos = angles::normalize_angle(right_pos);
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = current_time;
    joint_state.name.push_back("wheel_left_joint");
    joint_state.name.push_back("wheel_right_joint");
    joint_state.position.push_back(left_pos);
    joint_state.position.push_back(right_pos);
    joint_state_pub.publish(joint_state);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_publisher");
    OdomPublisher odom_publisher;
    ros::spin();
    return 0;
}