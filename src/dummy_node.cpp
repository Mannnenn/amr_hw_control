#include "amr_hw_control/dummy_node.hpp"

DummyNode::DummyNode() {
    sub = nh.subscribe("motor_command", 1, &DummyNode::callback, this);
    pub = nh.advertise<std_msgs::Float32MultiArray>("motor_response", 1);
}

void DummyNode::callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    pub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dummy_node");
    DummyNode node;
    ros::spin();
    return 0;
}