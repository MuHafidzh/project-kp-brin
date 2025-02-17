#include "dynamixel_workbench_controllers/dynamixel_workbench_controllers.h"

#include <keyboard/keyb.h>
#include <vector>
#include <unordered_map>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64.h>
#include <custom_msg/yolo.h>

#define MANUAL 0
#define CALIB 1
#define AUTO 2
#define NAV 3

#define VELOCITY 1
#define POSITION 3

class DynamixelBehaviorControl
{
public:
  DynamixelBehaviorControl()
  {
    client_ = nh_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");

    sub_ = nh_.subscribe("/dynamixel_workbench/dynamixel_state", 100, &DynamixelBehaviorControl::stateCallback, this);

    key_sub = nh_.subscribe("key", 100, &DynamixelBehaviorControl::keyCallback, this);

    gpio_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("gpio_control", 10);

    sub_object_detection = nh_.subscribe("yolo_topic", 1, &DynamixelBehaviorControl::objectDetectionCallback, this);

    servo_mode_pub_ = nh_.advertise<std_msgs::UInt8>("/dynamixel_workbench/mode", 10);

    nav_path_sub = nh_.subscribe("/path_radial", 1, &DynamixelBehaviorControl::navPathCallback, this);

    // Initialize flags
    key_flags_ = std::vector<bool>(8, false);
    key_status_ = std::vector<uint8_t>(42, 0); // Initialize key status

    // Initialize timer with 50 Hz frequency
    timer_ = nh_.createTimer(ros::Duration(0.02), &DynamixelBehaviorControl::timerCallback, this);

    gpio_msg_nav.data.resize(8);
    gpio_msg.data.resize(8);

    for (int i = 0; i < 8; ++i) {
        gpio_msg_nav.data[i] = 0;
        gpio_msg.data[i] = 0;
    }
  }

  void stateCallback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr& msg)
  {
    for (const auto& state : msg->dynamixel_state)
    {
      // Store the current position for ID 8 and 9
      if (state.id == 8) //pan 1500 kanan 2048 0 (tengah) 3500 kiri
      {
        position_8_ = state.present_position;
      }
      else if (state.id == 9) //tilt 1023 belakang 2048 0 (tengah) 3000 depan
      {
        position_9_ = state.present_position;
      }
    }
  }

  void objectDetectionCallback(const custom_msg::yolo::ConstPtr &msg)
  {
    class_array = msg->class_id;
    center_x_array = msg->center_x;
    center_y_array = msg->center_y;

    // printf("String Length: %zu\n", class_array.size());

    // for(int i = 0; i < class_array.size(); i++)
    //   printf("Object %d: %s, center_x: %f, center_y: %f\n", i, class_array[i].c_str(), center_x_array[i], center_y_array[i]);
    // printf("center_x_array: %f", center_x_array[0]);
    // ROS_INFO("Object %d: %s", i, msg->detection[i].object_name.c_str());
  }

  void navPathCallback(const std_msgs::Float64::ConstPtr &path_msg) //sub dalam derajat
  {       //0 deg      
  //      n               
  //90deg |    -90 deg
  // w ___|___ e    
  //      |
  //      |
  //      s
    //mapping gpio 
    /*
      0 dan 7 maju  (utara) -22.5 to 22.5
      3 dan 4 mundur (selatan) 157.5 to -157.5
      1 dan 2 kiri (barat) 67.5 to 112.5
      5 dan 6 kanan (timur) -67.5 to -112.5
      0 dan 1 (barat laut) 22.5 to 67.5
      6 dan 7 (timur laut) -22.5 to -67.5
      2 dan 3 (barat daya) 112.5 to 157.5
      4 dan 5 (tenggara) -112.5 to -157.5
    */
    nav_path_deg = path_msg->data;
  }

  void keyCallback(const keyboard::keyb::ConstPtr& msg) {
    key_status_ = msg->keys;

        // {0, '0'}, {1, '1'}, {2, '2'}, {3, '3'}, {4, '4'}, {5, '5'}, {6, '6'}, {7, '7'}, {8, '8'}, {9, '9'},
        // {10, 'a'}, {11, 'b'}, {12, 'c'}, {13, 'd'}, {14, 'e'}, {15, 'f'}, {16, 'g'}, {17, 'h'}, {18, 'i'}, {19, 'j'},
        // {20, 'k'}, {21, 'l'}, {22, 'm'}, {23, 'n'}, {24, 'o'}, {25, 'p'}, {26, 'q'}, {27, 'r'}, {28, 's'}, {29, 't'},
        // {30, 'u'}, {31, 'v'}, {32, 'w'}, {33, 'x'}, {34, 'y'}, {35, 'z'}, {36, '+'}, {37, '-'},
        // {38, 'U'}, {39, 'D'}, {40, 'R'}, {41, 'L'} // Up, Down, Right, Left arrows

    if(mode == MANUAL)
    {
      for (size_t i = 0; i < 8; ++i) {
        if (key_status_[i]) {
          key_flags_[i] = !key_flags_[i];
        }
        gpio_msg.data[i] = key_flags_[i] ? 1 : 0;
      }
      gpio_pub_.publish(gpio_msg);
    }

    

    // Publish new mode when 'k' or 'p' is pressed
    std_msgs::UInt8 servo_mode_msg;
    if (key_status_[20] == 1) // 'k' key
    {
      servo_mode_msg.data = VELOCITY; // Velocity control mode
      servo_mode_pub_.publish(servo_mode_msg);
      operating_mode = VELOCITY;
      ROS_INFO("Published Velocity Control Mode");
    }
    else if (key_status_[25] == 1) // 'p' key
    {
      servo_mode_msg.data = POSITION; // Position control mode
      servo_mode_pub_.publish(servo_mode_msg);
      operating_mode = POSITION;
      ROS_INFO("Published Position Control Mode");
    }
  }

  void timerCallback(const ros::TimerEvent&)
  {    
    //mode
    if(key_status_[22] == 1) //m
    {
      mode = MANUAL;
      printf("Manual Mode\n");
    }
    else if(key_status_[23] == 1) //n
    {
      mode = CALIB;
      printf("Calib Mode\n");
    }
    else if(key_status_[24] == 1) //o
    {
      mode = AUTO;
      printf("Auto Mode\n");
    }
    else if(key_status_[8] == 1) //8
    {
      mode = NAV;
      printf("tes gpio\n");
    }

    switch (mode)
    {
      case MANUAL:
        if(operating_mode == VELOCITY)
        {
          if(key_status_[39] == 1)
          {
            sendDynamixelCommand(9, "Goal_Velocity", 50);
          }
          else if(key_status_[38] == 1)
          {
            sendDynamixelCommand(9, "Goal_Velocity", -50);
          }
          else
          {
            sendDynamixelCommand(9, "Goal_Velocity", 0);
          }

          if(key_status_[40] == 1)
          {
            sendDynamixelCommand(8, "Goal_Velocity", -50);
          }
          else if(key_status_[41] == 1)
          {
            sendDynamixelCommand(8, "Goal_Velocity", 50);
          }
          else
          {
            sendDynamixelCommand(8, "Goal_Velocity", 0);
          }
        }
        else if(operating_mode == POSITION)
        {
          if (key_status_[28] == 1) // 'w' key
          {
            sendDynamixelCommand(9, "Goal_Position", position_9_ + 50);
          }
          else if (key_status_[32] == 1) // 's' key
          {
            sendDynamixelCommand(9, "Goal_Position", position_9_ - 50);
          }

          if (key_status_[10] == 1) // 'd' key
          {
            sendDynamixelCommand(8, "Goal_Position", position_8_ + 20);
          }
          else if (key_status_[13] == 1) // 'a' key
          {
            sendDynamixelCommand(8, "Goal_Position", position_8_ - 20);
          }        
        }
        break;
      case CALIB:
        // pid servo by something target position with velocity and position limitiation
        target[0] = 2048;
        target[1] = 2048;
        speed[0] = 10;
        speed[1] = 10;
        pid_servo(target, speed);
        
        if(fabs(target[0] - position_8_) < 10 && fabs(target[1] - position_9_) < 10)
        {
          servo_speed[0] = 0;
          servo_speed[1] = 0;
          
          //change mode to manual after 2 seconds
          if(++time_counter > 100)
          {
            printf("Change to Manual Mode\n");
            mode = MANUAL;
            time_counter = 0;
          }
          else
          {
            printf("Time Counter: %d\n", time_counter);
          }   
        }
        
        sendDynamixelCommand(8, "Goal_Velocity", servo_speed[0]);
        sendDynamixelCommand(9, "Goal_Velocity", servo_speed[1]);
        break;
      case AUTO:
        if(class_array.size() == 0)
        {
          if(++time_counter > 100)
          {
            target[0] = 2048;
            target[1] = 2048;
            speed[0] = 10;
            speed[1] = 10;
            pid_servo(target, speed);

            if(fabs(target[0] - position_8_) < 10 && fabs(target[1] - position_9_) < 10)
            {
              servo_speed[0] = 0;
              servo_speed[1] = 0;
            }

          }
          else
          {
            servo_speed[0] = 0;
            servo_speed[1] = 0;
            printf("Time Counter: %d\n", time_counter);
          }

          sendDynamixelCommand(8, "Goal_Velocity", servo_speed[0]);
          sendDynamixelCommand(9, "Goal_Velocity", servo_speed[1]);
        }
        //scan kanan kiri
        else
        {
          time_counter = 0;
          //jika botol
          for(int i = 0; i < class_array.size(); i++)
          {
            if(class_array[i] == "Bottle")
            {
              time_counter = 0;
              //x y 640 480
              xcam = center_x_array[i]; 
              ycam = center_y_array[i]; 

              pid_cam(xcam, ycam, 30, 30);

              if(fabs(err_pan) < 10 && fabs(err_tilt) < 10)
              {
                printf("Centered\n");
                speed_pan = 0;
                speed_tilt = 0;
              }

              sendDynamixelCommand(8, "Goal_Velocity", speed_pan);
              sendDynamixelCommand(9, "Goal_Velocity", speed_tilt);
            }
            else
            {
              sendDynamixelCommand(8, "Goal_Velocity", 0);
              sendDynamixelCommand(9, "Goal_Velocity", 0);
            }
          }
          
          
        }
        break;
      case NAV:    
        if (nav_path_deg >= -22.5 && nav_path_deg <= 22.5) //north
        {
          gpio_msg_nav.data[0] = 1;
          gpio_msg_nav.data[7] = 1;
          gpio_msg_nav.data[1] = 0;
          gpio_msg_nav.data[2] = 0;
          gpio_msg_nav.data[3] = 0;
          gpio_msg_nav.data[4] = 0;
          gpio_msg_nav.data[5] = 0;
          gpio_msg_nav.data[6] = 0;
        } 
        else if(nav_path_deg > 22.5 && nav_path_deg < 67.5) //north west
        {
          gpio_msg_nav.data[0] = 1;
          gpio_msg_nav.data[1] = 1;
          gpio_msg_nav.data[2] = 0;
          gpio_msg_nav.data[3] = 0;
          gpio_msg_nav.data[4] = 0;
          gpio_msg_nav.data[5] = 0;
          gpio_msg_nav.data[6] = 0;
          gpio_msg_nav.data[7] = 0;
        }
        else if(nav_path_deg >= 67.5 && nav_path_deg <= 112.5) //west
        {
          gpio_msg_nav.data[1] = 1;
          gpio_msg_nav.data[2] = 1;
          gpio_msg_nav.data[0] = 0;
          gpio_msg_nav.data[3] = 0;
          gpio_msg_nav.data[4] = 0;
          gpio_msg_nav.data[5] = 0;
          gpio_msg_nav.data[6] = 0;
          gpio_msg_nav.data[7] = 0;
        }
        else if(nav_path_deg > 112.5 && nav_path_deg < 157.5) //south west
        {
          gpio_msg_nav.data[2] = 1;
          gpio_msg_nav.data[3] = 1;
          gpio_msg_nav.data[0] = 0;
          gpio_msg_nav.data[1] = 0;
          gpio_msg_nav.data[4] = 0;
          gpio_msg_nav.data[5] = 0;
          gpio_msg_nav.data[6] = 0;
          gpio_msg_nav.data[7] = 0;
        }
        else if(nav_path_deg >= 157.5 || nav_path_deg <= -157.5) //south
        {
          gpio_msg_nav.data[3] = 1;
          gpio_msg_nav.data[4] = 1;
          gpio_msg_nav.data[0] = 0;
          gpio_msg_nav.data[1] = 0;
          gpio_msg_nav.data[2] = 0;
          gpio_msg_nav.data[5] = 0;
          gpio_msg_nav.data[6] = 0;
          gpio_msg_nav.data[7] = 0;
        }
        else if(nav_path_deg > -157.5 && nav_path_deg < -112.5) //south east
        {
          gpio_msg_nav.data[4] = 1;
          gpio_msg_nav.data[5] = 1;
          gpio_msg_nav.data[0] = 0;
          gpio_msg_nav.data[1] = 0;
          gpio_msg_nav.data[2] = 0;
          gpio_msg_nav.data[3] = 0;
          gpio_msg_nav.data[6] = 0;
          gpio_msg_nav.data[7] = 0;
        }
        else if(nav_path_deg >= -112.5 && nav_path_deg <= -67.5) //east
        {
          gpio_msg_nav.data[5] = 1;
          gpio_msg_nav.data[6] = 1;
          gpio_msg_nav.data[0] = 0;
          gpio_msg_nav.data[1] = 0;
          gpio_msg_nav.data[2] = 0;
          gpio_msg_nav.data[3] = 0;
          gpio_msg_nav.data[4] = 0;
          gpio_msg_nav.data[7] = 0;
        }
        else if(nav_path_deg > -67.5 && nav_path_deg < -22.5) //north east
        {
          gpio_msg_nav.data[6] = 1;
          gpio_msg_nav.data[7] = 1;
          gpio_msg_nav.data[0] = 0;
          gpio_msg_nav.data[1] = 0;
          gpio_msg_nav.data[2] = 0;
          gpio_msg_nav.data[3] = 0;
          gpio_msg_nav.data[4] = 0;
          gpio_msg_nav.data[5] = 0;
        }

        gpio_pub_.publish(gpio_msg_nav);
        break;  
    } 
  }

  void pid_servo(float tar[2], float set_speed[2])
  {
    float error[2];
    float sum_error[2];
    float prev_error[2];
    float speed_track[2];
    float P[2], I[2], D[2];
    float kp[2] = {0.1, 0.1};
    float Ti[2] = {0.0, 0.0};
    float Td[2] = {0.0, 0.0};
    float dt = 0.02;

    error[0] = target[0] - position_8_;
    error[1] = target[1] - position_9_;

    speed_track[0] = kp[0] * error[0];
    speed_track[1] = kp[1] * error[1];

    if(speed_track[0] > set_speed[0])
      speed_track[0] = set_speed[0];
    else if(speed_track[0] < -set_speed[0])
      speed_track[0] = -set_speed[0];

    if(speed_track[1] > set_speed[1])
      speed_track[1] = set_speed[1];
    else if(speed_track[1] < -set_speed[1])
      speed_track[1] = -set_speed[1];

    servo_speed[0] = speed_track[0];
    servo_speed[1] = speed_track[1];
  }

  void pid_cam(float px_x, float px_y, float spd_pan, float spd_tilt)
  {
    err_pan = 320 - px_x;
    err_tilt = px_y - 240;

    sum_err_pan += err_pan;
    sum_err_tilt += err_tilt;

    P_pan = kp_pan * err_pan;
    I_pan = Ti_pan * sum_err_pan;
    D_pan = Td_pan * (err_pan - prev_err_pan) / dt;

    P_tilt = kp_tilt * err_tilt;
    I_tilt = Ti_tilt * sum_err_tilt;
    D_tilt = Td_tilt * (err_tilt - prev_err_tilt) / dt;

    speed_pan = P_pan + I_pan + D_pan;
    speed_tilt = P_tilt + I_tilt + D_tilt;
    
    if(speed_pan > spd_pan)
      speed_pan = spd_pan;
    else if(speed_pan < -spd_pan)
      speed_pan = -spd_pan;
    
    if(speed_tilt > spd_tilt)
      speed_tilt = spd_tilt;
    else if(speed_tilt < -spd_tilt)
      speed_tilt = -spd_tilt;

    prev_err_pan = err_pan;
    prev_err_tilt = err_tilt;
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
      // ROS_ERROR("Failed to call service DynamixelCommand");
      return false;
    }
  }



private:
  ros::NodeHandle nh_;
  ros::ServiceClient client_;
  ros::Subscriber sub_;
  ros::Subscriber key_sub;
  ros::Subscriber sub_object_detection;
  ros::Subscriber nav_path_sub;
  ros::Publisher gpio_pub_;
  ros::Publisher servo_mode_pub_;
  ros::Timer timer_;
  std::vector<bool> key_flags_;
  std::vector<uint8_t> key_status_;
  int position_8_ = 0;
  int position_9_ = 0;
  uint8_t mode = MANUAL;
  uint8_t operating_mode = 1;
  float target[2], speed[2];
  float servo_speed[2];
  float xcam, ycam;
  std::vector<float> center_x_array, center_y_array;
  std::vector<std::string> class_array;
  double nav_path_deg = 0;

  uint32_t time_counter = 0;

  std_msgs::Int32MultiArray gpio_msg_nav;
  std_msgs::Int32MultiArray gpio_msg;

  //komponen pid cam servo
  float err_pan, err_tilt;
  float sum_err_pan, sum_err_tilt;
  float prev_err_pan, prev_err_tilt;
  float speed_pan, speed_tilt;
  float P_pan, I_pan, D_pan;
  float P_tilt, I_tilt, D_tilt;
  float kp_pan = 0.1, kp_tilt = 0.1;
  float Ti_pan = 0.0, Ti_tilt = 0.0;
  float Td_pan = 0.0, Td_tilt = 0.0;
  float dt = 0.02;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "coba");
  DynamixelBehaviorControl dbc;
  ros::spin();
  return 0;
}