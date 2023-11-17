#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

void control(const sensor_msgs::Joy::ConstPtr& msg, ros::Publisher twist_pub) {
    float L_horizontal = msg->axes[0]; // 左ジョイスティック（左右）
    float L_vertical = msg->axes[1]; // 左ジョイスティック（上下）

    int front = msg->buttons[2]; //triangle
    int back = msg->buttons[0]; //cross
    int right = msg->buttons[1]; //circle
    int left = msg->buttons[3]; //square

    std::vector<float> velocity = {L_horizontal, L_vertical};

    geometry_msgs::Twist t; // Twistのインスタンスを生成

    t.angular.z = velocity[0] - right + left;
    t.linear.x = velocity[1] * 0.75 + ((float)front) / 4 - ((float)back) / 4; // twistにjoyから取得したデータを当てはめる

    twist_pub.publish(t); // twistを配信
}

int main(int argc, char** argv) {
    // ノードの初期化
    ros::init(argc, argv, "joy_to_twist");
    ros::NodeHandle nh;
    // 配信準備
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    // 購読
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, boost::bind(control, _1, twist_pub));

    ros::spin();
    return 0;
}