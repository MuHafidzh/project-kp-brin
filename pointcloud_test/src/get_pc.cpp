#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <custom_msg/yolo.h>
#include <vector>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <stdint.h>
#include <deque>
#include <numeric>
#include <stdexcept>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

std::vector<float> center_x_array, center_y_array;
std::vector<std::string> class_array;
float pc_x, pc_y;
std::string class_name, user_command;
ros::Publisher pc_depth_pub;
std_msgs::Float32 pc_depth_msg;
float pc_depth_ave;

template <typename T>
class Moving_Average
{
    std::deque<T> buffer;
    uint16_t len;
    T output;
    T last_input;

public:
    Moving_Average(uint16_t length) : len(length) {}

    T update_on_change(T input)
    {
        static T last_input;
        if (input != last_input)
        {
            last_input = input;
            return update(input);
        }
        else
            return output;
    }

    T update_sync(T input)
    {
        if (input != last_input)
        {
            last_input = input;
            return update(input);
        }
        else
        {
            return output;
        }
    }

    T update(T input)
    {
        if (buffer.size() >= len)
        {
            buffer.pop_front();
        }
        buffer.push_back(input);
        output = std::accumulate(buffer.begin(), buffer.end(), 0.0) / buffer.size();
        return output;
    }

    void fill_buffer(T value)
    {
        buffer.clear();
        buffer.resize(len, value);
        update(value);
    }

    size_t length() { return len; }

    T out() { return output; }
};

Moving_Average<float> PcDepthAve(20);

bool flag_once;
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{
    // // ROS_INFO("Received point cloud message");

    // // Iterate over the point cloud
    // sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    // sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
    // sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

    // float min_distance = std::numeric_limits<float>::max();
    // float depth = std::numeric_limits<float>::quiet_NaN();

    // if (center_x_array.size() != 0 && center_y_array.size() != 0)
    // {
    //     for (int i = 0; i < class_array.size(); ++i)
    //     {
    //         pc_x = center_x_array[i] / 2; // divide by 2 because depth image is 320 x 240 :(
    //         pc_y = center_y_array[i] / 2;
    //         class_name = class_array[i];
    //         if (class_name != user_command)
    //             return;

    //         for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    //         {
    //             float distance = std::sqrt(std::pow(*iter_x - pc_x, 2) + std::pow(*iter_y - pc_y, 2));
    //             if (distance < min_distance)
    //             {
    //                 min_distance = distance;
    //                 depth = *iter_z;
    //             }
    //         }
    //         // ROS_INFO("'%s' Depth at coordinates (%f, %f): %f", class_name.c_str(), pc_x, pc_y, depth);
    //         pc_depth_ave = PcDepthAve.update(depth);
    //         ROS_INFO("'%s' Depth at coordinates (%f, %f): %f", class_name.c_str(), pc_x, pc_y, pc_depth_ave);

    //         pc_depth_msg.data = depth;
    //         pc_depth_pub.publish(pc_depth_msg);
    //     }
    // }
    // else
    // {
    //     ROS_INFO("No object detected");
    // }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");
    // for (int i = 0; i < 100000; i++)
    // {
    //     iter_z += 1;
    //     try
    //     {
    //         float depth = *iter_z;
    //         printf("%d\n", i);
    //         ROS_INFO("height: %d, width: %d", cloud_msg->height, cloud_msg->width);
    //         ROS_INFO("row_step: %d, point_step: %d", cloud_msg->row_step, cloud_msg->point_step);
    //     }
    //     catch (std::runtime_error &e)
    //     {

    //         ROS_INFO("Index: %d Error: %s", i, e.what());
    //     }
    // }
    if (flag_once == false)
    {
        for (int i = 5000; i < 10000; i += 5)
        {
            // ROS_INFO("I: %d X: %f", i, iter_x[i]);
        }
        flag_once = true;
    }
    // ROS_INFO("Depth at coordinates (0, 0): %f", depth);
}

// Mat depth_image = Mat::zeros(Size(640, 480), CV_16UC1);
void depthCallback(const sensor_msgs::ImageConstPtr &msg)
{

    if (class_array.empty() || user_command.empty())
        return;

    for (int i = 0; i < class_array.size(); ++i)
    {
        if (class_array[i] == user_command)
        {
            pc_x = center_x_array[i];
            pc_y = center_y_array[i];
            class_name = class_array[i];

            // Convert ROS image to OpenCV format
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

            uint16_t x = (uint16_t)pc_x;
            uint16_t y = (uint16_t)pc_y;
            // Get the depth value at (x, y)
            uint16_t depth_value = cv_ptr->image.at<uint16_t>(y, x);

            float depth_float = (float)depth_value / 1000.0; // Convert to meters
            // Print depth value in millimeters
            ROS_INFO("'%s' Depth at coordinates (%d, %d): %d", class_name.c_str(), x, y, depth_value);
	    if(depth_float != 0){
            	pc_depth_msg.data = depth_float; // float32
            	pc_depth_pub.publish(pc_depth_msg);
	    }
        }
    }
}

void userCommandCallback(const std_msgs::String::ConstPtr &msg)
{
    user_command = msg->data;
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
    // for testing purpose, turn off if get data from yolo
    // center_x_array.push_back(320.0);
    // center_y_array.push_back(240.0);
    // class_array.push_back("person");
    ros::Subscriber sub_user_command = nh.subscribe("/user_command", 1, userCommandCallback);
    // ros::Subscriber sub_pointcloud = nh.subscribe("/camera/depth/color/points", 1, pointCloudCallback);
    ros::Subscriber sub_object_detection = nh.subscribe("yolo_topic", 1, objectDetectionCallback);
    ros::Subscriber aligned_depth_sub = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depthCallback);

    pc_depth_pub = nh.advertise<std_msgs::Float32>("/pc_depth", 1);

    ros::spin();

    return 0;
}
