#include "amr_hw_control/twist_to_speed.hpp"

Twist2Float32::Twist2Float32() {
    command_left.data = 0.0;
    command_right.data = 0.0;
    received_twist = nullptr;
    ros::NodeHandle nh;
    sub = nh.subscribe("/cmd_vel", 1, &Twist2Float32::callback, this);
    pub_right = nh.advertise<std_msgs::Float32>("right_motor/cmd_vel", 10);
    pub_left = nh.advertise<std_msgs::Float32>("left_motor/cmd_vel", 10);
}

void Twist2Float32::callback(const geometry_msgs::Twist::ConstPtr& message) {
    received_twist = message;
    std::tie(command_right.data, command_left.data) = twist2rpm(*received_twist);
    pub_right.publish(command_right);
    pub_left.publish(command_left);
}

std::tuple<float, float> Twist2Float32::twist2rpm(const geometry_msgs::Twist& received_data) {
    float wheel_size;
    float axle_length;
    int gear_ratio;

    ros::NodeHandle nh;
    nh.param<float>("wheel_size", wheel_size, 0.075);
    nh.param<float>("axle_length", axle_length, 0.35);
    nh.param<int>("gear_ratio", gear_ratio, 19);

    float v = received_data.linear.x; // (m/s)
    float omega = received_data.angular.z; // (rad/s)

    float v_r = (omega * axle_length + 2 * v) / 2;
    float v_l = (omega * axle_length - 2 * v) / (-2);

    v_r = v_r / (wheel_size * 2 * 3.14); // wheel_speed(1/s)
    v_l = v_l / (wheel_size * 2 * 3.14); // wheel_speed(1/s)
    float r_rpm = 60 * v_r * gear_ratio; // gear rate
    float l_rpm = 60 * v_l * gear_ratio; // gear rate

    return std::make_tuple(r_rpm, l_rpm);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Twist2Float32");
    Twist2Float32 convert;
    ros::spin();
    return 0;
}