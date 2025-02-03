#include "dynamixel_workbench_controllers/dynamixel_workbench_controllers.h"

#include <keyboard/keyb.h>
#include <vector>
#include <unordered_map>
#include <std_msgs/Int32MultiArray.h>

class DynamixelBehaviorControl
{
public:
  DynamixelBehaviorControl()
  {
    client_ = nh_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");

    sub_ = nh_.subscribe("/dynamixel_workbench/dynamixel_state", 100, &DynamixelBehaviorControl::stateCallback, this);

    key_sub = nh_.subscribe("key", 100, &DynamixelBehaviorControl::keyCallback, this);

    gpio_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("gpio_control", 10);

    // Initialize flags
    key_flags_ = std::vector<bool>(8, false);
    key_status_ = std::vector<uint8_t>(42, 0); // Initialize key status

    // Initialize timer with 50 Hz frequency
    timer_ = nh_.createTimer(ros::Duration(0.02), &DynamixelBehaviorControl::timerCallback, this);
  }

  void stateCallback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr& msg)
  {
    for (const auto& state : msg->dynamixel_state)
    {
      ROS_INFO("ID: %d, Position: %d, Velocity: %d, Current: %d", state.id, state.present_position, state.present_velocity, state.present_current);

      // Ensure position is within bounds
      if (state.present_position < 0 || state.present_position > 4096)
      {
        ROS_WARN("ID: %d position out of bounds: %d", state.id, state.present_position);
        continue;
      }

      // Store the current position for ID 8 and 9
      if (state.id == 8)
      {
        position_8_ = state.present_position;
      }
      else if (state.id == 9)
      {
        position_9_ = state.present_position;
      }
    }
  }

  void keyCallback(const keyboard::keyb::ConstPtr& msg) {
    key_status_ = msg->keys;

    std_msgs::Int32MultiArray gpio_msg;
    gpio_msg.data.resize(8);

    for (size_t i = 0; i < 8; ++i) {
      if (key_status_[i]) {
        key_flags_[i] = !key_flags_[i];
      }
      gpio_msg.data[i] = key_flags_[i] ? 1 : 0;
    }

    gpio_pub_.publish(gpio_msg);
  }

  void timerCallback(const ros::TimerEvent&)
  {
    // Control Dynamixel based on arrow keys and position limits
    if (key_status_[38] && position_9_ < 4096) { // Up arrow
      sendDynamixelCommand(9, "Goal_Velocity", -50);
    } else if (key_status_[39] && position_9_ > 0) { // Down arrow
      sendDynamixelCommand(9, "Goal_Velocity", 50);
    } else {
      sendDynamixelCommand(9, "Goal_Velocity", 0);
    }

    if (key_status_[40] && position_8_ < 4096) { // Right arrow
      sendDynamixelCommand(8, "Goal_Velocity", 50);
    } else if (key_status_[41] && position_8_ > 0) { // Left arrow
      sendDynamixelCommand(8, "Goal_Velocity", -50);
    } else {
      sendDynamixelCommand(8, "Goal_Velocity", 0);
    }
  }

  bool sendDynamixelCommand(uint8_t id, const std::string &addr_name, int32_t value)
  {
    dynamixel_workbench_msgs::DynamixelCommand srv;
    srv.request.command = "";
    srv.request.id = id;
    srv.request.addr_name = addr_name;
    srv.request.value = value;

    if (client_.call(srv))
    {
      return srv.response.comm_result;
    }
    else
    {
      ROS_ERROR("Failed to call service DynamixelCommand");
      return false;
    }
  }

private:
  ros::NodeHandle nh_;
  ros::ServiceClient client_;
  ros::Subscriber sub_;
  ros::Subscriber key_sub;
  ros::Publisher gpio_pub_;
  ros::Timer timer_;
  std::vector<bool> key_flags_;
  std::vector<uint8_t> key_status_;
  int position_8_ = 0;
  int position_9_ = 0;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "coba");
  DynamixelBehaviorControl dbc;
  ros::spin();
  return 0;
}