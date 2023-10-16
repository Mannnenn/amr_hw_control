#include "amr_hw_control/twist_to_speed.hpp"

Twist2Float32::Twist2Float32() {
    motor_command.data = {0,0};
    received_twist = nullptr;
    ros::NodeHandle nh;
    sub = nh.subscribe("/cmd_vel", 1, &Twist2Float32::callback, this);
    pub_motor_command = nh.advertise<std_msgs::Float32MultiArray>("motor_command", 10);
}

void Twist2Float32::callback(const geometry_msgs::Twist::ConstPtr& message) {
    received_twist = message;
    std::tie(motor_command.data[0], motor_command.data[1]) = twist2vel(*received_twist);
    pub_motor_command.publish(motor_command);
}

std::tuple<float, float> Twist2Float32::twist2vel(const geometry_msgs::Twist& received_data) {
    float wheel_size;
    float axle_length;
    ros::NodeHandle nh;
    nh.param<float>("wheel_size", wheel_size, 1.0);
    nh.param<float>("axle_length", axle_length, 1.0);

    float v = received_data.linear.x; // (m/s)
    float omega = received_data.angular.z; // (rad/s)

    float v_r = omega * (axle_length / 2) + v; // m/s
    float v_l = - omega * (axle_length / 2) + v; // m/s

    v_r = v_r / (wheel_size / 2); // wheel_speed(rad/s)
    v_l = v_l / (wheel_size / 2); // wheel_speed(rad/s)

    return std::make_tuple(v_r, v_l);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Twist2Float32");
    Twist2Float32 convert;
    ros::spin();
    return 0;
}