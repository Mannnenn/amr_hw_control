#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

class Twist2Float32 {
public:
    Twist2Float32() {
        command_left.data = 0.0;
        command_right.data = 0.0;
        received_twist = nullptr;
        ros::NodeHandle nh;
        sub = nh.subscribe("/cmd_vel", 1, &Twist2Float32::callback, this);
        pub_right = nh.advertise<std_msgs::Float32>("right_motor/cmd_vel", 10);
        pub_left = nh.advertise<std_msgs::Float32>("left_motor/cmd_vel", 10);
    }

    void main_twist2Float32() {
        ros::spin();
    }

    void callback(const geometry_msgs::Twist::ConstPtr& message) {
        received_twist = message;
        std::tie(command_right.data, command_left.data) = twist2rpm(*received_twist);
        pub_right.publish(command_right);
        pub_left.publish(command_left);
    }

    std::tuple<float, float> twist2rpm(const geometry_msgs::Twist& received_data) {
        float wheeles_size = 0.075; // wheel size
        float axle_length = 0.35; // axle_size(2d)

        float v = received_data.linear.x; // (m/s)
        float omega = received_data.angular.z; // (rad/s)

        float v_r = (omega * axle_length + 2 * v) / 2;
        float v_l = (omega * axle_length - 2 * v) / (-2);

        v_r = v_r / (wheeles_size * 2 * 3.14); // wheel_speed(1/s)
        v_l = v_l / (wheeles_size * 2 * 3.14); // wheel_speed(1/s)
        float r_rpm = 60 * v_r * 19; // gear rate
        float l_rpm = 60 * v_l * 19; // gear rate

        return std::make_tuple(r_rpm, l_rpm);
    }

private:
    std_msgs::Float32 command_left;
    std_msgs::Float32 command_right;
    geometry_msgs::Twist::ConstPtr received_twist;
    ros::Subscriber sub;
    ros::Publisher pub_right;
    ros::Publisher pub_left;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "Twist2Float32");
    Twist2Float32 convert;
    convert.main_twist2Float32();
    return 0;
}