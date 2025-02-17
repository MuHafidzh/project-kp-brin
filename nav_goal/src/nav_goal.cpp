#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <chrono>
#include <std_msgs/Float32.h>
#include "dynamixel_workbench_controllers/dynamixel_workbench_controllers.h"

#define CONTROL_LOOP 0.5

ros::Publisher goal_pub;
ros::Subscriber path_sub;
float pc_depth;
float theta_servo;

void loop(const ros::TimerEvent &event)
{
    // if (!ros::ok())
    //     return;

    // Wait for the publisher to connect to the topic

    if (goal_pub.getNumSubscribers() == 0)
    {
        ROS_INFO("Waiting for subscribers...");
        return;
    }

    // convert depth and theta to x and y
    theta_servo = theta_servo * 3.14159265359 / 180;
    float nav_x = pc_depth * cos(theta_servo);
    float nav_y = pc_depth * sin(theta_servo);

    // multiply by -1 because default is inverted
    nav_x = -nav_x;
    nav_y = -nav_y;

    ROS_INFO("nav_x: %f, nav_y: %f", nav_x, nav_y);

    // Create the PoseStamped message
    geometry_msgs::PoseStamped goal_msg;

    // Set the header
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.header.frame_id = "map";

    // Set the position (x, y, z)
    goal_msg.pose.position.x = nav_x;
    goal_msg.pose.position.y = nav_y;
    goal_msg.pose.position.z = 0.0;

    // Set the orientation (w = 1.0 for no rotation)
    goal_msg.pose.orientation.w = 1.0;
    // goal_msg.pose.orientation.x = 1.0;

    // Publish the message
    goal_pub.publish(goal_msg);
    ROS_INFO("Goal published to /move_base_simple/goal");
}

void pcDepthCallback(const std_msgs::Float32::ConstPtr &pc_depth_msg)
{
    // ROS_INFO("Depth: %f", pc_depth_msg->data);
    pc_depth = pc_depth_msg->data;
}

void stateCallback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr &msg)
{
    float servo_digital;
    for (const auto &state : msg->dynamixel_state)
    {
        // Store the current position for ID 8 and 9
        if (state.id == 8) // pan 1500 kanan 2048 0 (tengah) 3500 kiri
        {
            servo_digital = state.present_position;
        }
    }
    theta_servo = (servo_digital - 2048) * 360 / 4096;
}
int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "move_base_goal_publisher");
    ros::NodeHandle nh;

    // Create a publisher for the /move_base_simple/goal topic
    ros::Subscriber pc_depth_sub = nh.subscribe("/pc_depth", 1, pcDepthCallback);
    ros::Subscriber servo_sub = nh.subscribe("/dynamixel_workbench/dynamixel_state", 100, stateCallback);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    ros::Timer control_loop =
        nh.createTimer(ros::Duration(CONTROL_LOOP), loop);

    // Spin once to ensure the message is sent
    ros::spin();
    ros::waitForShutdown();

    return 0;
}
