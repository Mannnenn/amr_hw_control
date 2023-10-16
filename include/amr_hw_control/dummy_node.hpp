#ifndef DUMMY_NODE_HPP
#define DUMMY_NODE_HPP

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

class DummyNode {
public:
    DummyNode();
    void callback(const std_msgs::Float32MultiArray::ConstPtr& msg);

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
};

#endif // DUMMY_NODE_HPP