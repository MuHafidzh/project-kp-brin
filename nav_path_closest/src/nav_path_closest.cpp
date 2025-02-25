#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

ros::Publisher path_pub;
std_msgs::Float64 radial_msg;

void pathCallback(const nav_msgs::Path::ConstPtr &path_msg)
{
    if (path_msg->poses.empty())
    {
        ROS_WARN("Received empty path");
        return;
    }
    geometry_msgs::PoseStamped nav_pose;
    size_t path_size = path_msg->poses.size();
    ROS_INFO("Size of path_msg: %zu", path_size);
    if(path_size < 10){
        ROS_WARN("Path size less than 10");
        return;
    }

    nav_pose = path_msg->poses[10];
    // multiply by -1 because default is inverted
    nav_pose.pose.position.x = -nav_pose.pose.position.x;
    nav_pose.pose.position.y = -nav_pose.pose.position.y;

    ROS_INFO("First Pose: x=%.2f, y=%.2f", nav_pose.pose.position.x, nav_pose.pose.position.y);
    double path_radial;
    path_radial = atan2(nav_pose.pose.position.y, nav_pose.pose.position.x);
    path_radial = path_radial * 180 / 3.14159265359;
    // geometry_msgs::Quaternion orientation = nav_pose.pose.orientation;
    // ROS_INFO("Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f", orientation.x, orientation.y, orientation.z, orientation.w);

    radial_msg.data = path_radial;
    path_pub.publish(radial_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_path_closest");
    ros::NodeHandle nh;

    ros::Subscriber path_sub = nh.subscribe("/move_base/NavfnROS/plan", 1, pathCallback);
    path_pub = nh.advertise<std_msgs::Float64>("/path_radial", 1);

    ros::spin();
    return 0;
}