#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <custom_msg/yolo.h>
#include <vector>

std::vector<float> center_x_array, center_y_array;
std::vector<std::string> class_array;
float pc_x, pc_y;
std::string class_name;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{

    // Iterate over the point cloud
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

    float min_distance = std::numeric_limits<float>::max();
    float depth = std::numeric_limits<float>::quiet_NaN();

    if (center_x_array.size() != 0 && center_y_array.size() != 0)
    {
        for (int i = 0; i < class_array.size(); ++i)
        {
            pc_x = center_x_array[i];
            pc_y = center_y_array[i];
            class_name = class_array[i];

            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
            {
                float distance = std::sqrt(std::pow(*iter_x - pc_x, 2) + std::pow(*iter_y - pc_y, 2));
                if (distance < min_distance)
                {
                    min_distance = distance;
                    depth = *iter_z;
                }
            }

            ROS_INFO("'%s' Depth at coordinates (%f, %f): %f", class_name.c_str(), pc_x, pc_y, depth);
        }
    }
}

void objectDetectionCallback(const custom_msg::yolo::ConstPtr &msg)
{
    class_array = msg->class_id;
    center_x_array = msg->center_x;
    center_y_array = msg->center_y;
    // printf("center_x_array: %f", center_x_array[0]);
    // ROS_INFO("Object %d: %s", i, msg->detection[i].object_name.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub_pointcloud = nh.subscribe("/camera/depth/color/points", 1, pointCloudCallback);
    ros::Subscriber sub_object_detection = nh.subscribe("yolo_topic", 1, objectDetectionCallback);

    ros::spin();

    return 0;
}
