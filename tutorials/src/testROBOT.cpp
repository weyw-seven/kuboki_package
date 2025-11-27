#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <stdlib.h>
#include <iostream>

using namespace std;

vector<float> range_val;

const float threshold = 0.5;

void counterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // 720 --> 360degree
    // recommend to print out the whole structure of this topic.
    //ROS_INFO("subscribed");
    
    range_val=msg->ranges;
    //ROS_INFO("range_val's size = %d",range_val.size());
    //ROS_INFO("range_val[0]'s value = %f",range_val[0]);
    //ROS_INFO("range_val[180]'s value = %f",range_val[180]);
    //ROS_INFO("range_val[360]'s value = %f",range_val[360]);
    //ROS_INFO("range_val[540]'s value = %f",range_val[540]);
    if(range_val[0] < threshold)
    {
        ROS_INFO("object at front");
    }
    if(range_val[180] < threshold)
    {
        ROS_INFO("object at left");
    }
    if(range_val[360] < threshold)
    {
        ROS_INFO("object at back");
    }
    if(range_val[540] < threshold)
    {
        ROS_INFO("object at right");
    }
}

int main(int argc, char **argv)
{
    // initialize the ROS node, and name it "test_node", while ros::NodeHandle starts the node.
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;


    // Publishing a message is done using ros:Publisher pub=nh.advertise.
    // and followed by the message type that we are going to be sending. 
    // The 100 is the message queue size.
    // if you are publishing message faster then what roscpp can send, 
    // 100 messages will be saved in the queue to be sent. 
    // The larger the queue, the more delay in robot movement in case of buffering.
    ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 100);
    ros::Subscriber sub=nh.subscribe("/scan",1000,counterCallback);

    srand(time(0));

    ros::Rate rate(10);

    //The ros::ok function will return true unless it recevis a command to shut down, either by using the rosnode kill command, or by the user pusing Ctrl-C in a terminal.
    while(ros::ok())
    {
        //This creates the message we are going to send, msg, of the type geometry_msgs:Twist
        geometry_msgs::Twist msg;
        
    
        msg.linear.x = 0.1;
        msg.angular.z = 0.0;
        
        // adding msg to the publisher queue to be sent.
        pub.publish(msg);
        ROS_INFO("publishing");
        ros::spinOnce();

        // ROS is able to control the loop frequency using ros:Rate to dictate how rapidly the loop will run in Hz.
        // rate.sleep will delay a variable amount of time such that your loop cycles at the desired frequency.
        // This accounts for time consumed by other parts of the loop. 
        // All Clearpath robots require a minimum loop rate of 10Hz.
        rate.sleep();

        
    }
}
