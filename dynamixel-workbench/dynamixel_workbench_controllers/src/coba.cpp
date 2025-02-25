#include "dynamixel_workbench_controllers/dynamixel_workbench_controllers.h"

#include <keyboard/keyb.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <custom_msg/yolo.h>
#include <stdbool.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#define VELOCITY 1
#define POSITION 3

#define RIGHT 1
#define LEFT 2

#define AFK 0
#define MANUAL 1
#define CALIB 2
#define SCAN 3

class DynamixelBehaviorControl
{
public: 
  
  DynamixelBehaviorControl()
  {
    client_ = nh_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");

    sub_ = nh_.subscribe("/dynamixel_workbench/dynamixel_state", 100, &DynamixelBehaviorControl::stateCallback, this);
    key_sub = nh_.subscribe("key", 100, &DynamixelBehaviorControl::keyCallback, this);
    sub_object_detection = nh_.subscribe("yolo_topic", 1, &DynamixelBehaviorControl::objectDetectionCallback, this);
    nav_path_sub = nh_.subscribe("/path_radial", 1, &DynamixelBehaviorControl::navPathCallback, this);
    user_command_sub = nh_.subscribe("user_command", 1, &DynamixelBehaviorControl::userCommandCallback, this);
    pc_depth_sub = nh_.subscribe("/pc_depth", 1, &DynamixelBehaviorControl::pcDepthCallback, this);

    gpio_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("gpio_control", 10);
    servo_mode_pub_ = nh_.advertise<std_msgs::UInt8>("/dynamixel_workbench/mode", 10);
    pub_nav_flag = nh_.advertise<std_msgs::Bool>("nav_flag", 1);
    pub_speech = nh_.advertise<std_msgs::String>("out_speech", 1);

    // Initialize flags
    key_flags_ = std::vector<bool>(8, false);
    key_status_ = std::vector<uint8_t>(42, 0); // Initialize key status

    // Initialize timer with 50 Hz frequency
    timer_ = nh_.createTimer(ros::Duration(0.02), &DynamixelBehaviorControl::timerCallback, this);
    //timer gpio 0.5 Hz
    timer_gpio = nh_.createTimer(ros::Duration(0.25), &DynamixelBehaviorControl::timerGpioCallback, this);

    gpio_msg_nav.data.resize(8);
    gpio_msg.data.resize(8);

    for (int i = 0; i < 8; ++i) {
        gpio_msg_nav.data[i] = 0;
        gpio_msg.data[i] = 0;
    }
    gpio_pub_.publish(gpio_msg_nav);

    
    printf("PROGRAM STARTO\n");
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
    class_array = msg->class_id; //string
    center_x_array = msg->center_x;
    center_y_array = msg->center_y;
  }

  void navPathCallback(const std_msgs::Float64::ConstPtr &path_msg) //sub dalam derajat
  {    
    nav_path_deg_buf = path_msg->data;
  }

  void pcDepthCallback(const std_msgs::Float32::ConstPtr &pc_msg)
  {
    pc_depth = pc_msg->data; //point cloud depth
    //printf("pc_depth: %f\n", pc_depth); 
  }

  void keyCallback(const keyboard::keyb::ConstPtr& msg) {
    key_status_ = msg->keys;

        // {0, '0'}, {1, '1'}, {2, '2'}, {3, '3'}, {4, '4'}, {5, '5'}, {6, '6'}, {7, '7'}, {8, '8'}, {9, '9'},
        // {10, 'a'}, {11, 'b'}, {12, 'c'}, {13, 'd'}, {14, 'e'}, {15, 'f'}, {16, 'g'}, {17, 'h'}, {18, 'i'}, {19, 'j'},
        // {20, 'k'}, {21, 'l'}, {22, 'm'}, {23, 'n'}, {24, 'o'}, {25, 'p'}, {26, 'q'}, {27, 'r'}, {28, 's'}, {29, 't'},
        // {30, 'u'}, {31, 'v'}, {32, 'w'}, {33, 'x'}, {34, 'y'}, {35, 'z'}, {36, '+'}, {37, '-'},
        // {38, 'U'}, {39, 'D'}, {40, 'R'}, {41, 'L'} // Up, Down, Right, Left arrows

    // if(mode_nav == MANUAL)
    // {
    //   for (size_t i = 0; i < 8; ++i) {
    //     if (key_status_[i]) {
    //       key_flags_[i] = !key_flags_[i];
    //     }
    //     gpio_msg.data[i] = key_flags_[i] ? 1 : 0;
    //   }
    //   gpio_pub_.publish(gpio_msg);
    // }

    // Publish new mode when 'k' or 'p' is pressed
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

  void userCommandCallback(const std_msgs::String::ConstPtr &msg)
  {
      user_command = msg->data; //printah buat jalan
      if (!user_command.empty() && user_command == previous_user_command)
      {
          flag_command = false;
      }
      else
      {
          flag_command = true;
      }

      // Update the previous command
      previous_user_command = user_command;
  }

  void timerCallback(const ros::TimerEvent&)
  {        
    //mode berdasarkan user command
    if (flag_command)
    {
        for (const auto& target : object_targets)
        {
            if (user_command == target)
            {
                pubSpeech("scan " + user_command);
                mode_nav = SCAN;
                printf("Scan Mode\n");
                flag_command = false;
                resetSpeechFlag();
                return;
            }
        }

        if (user_command == "stop")
        {
            mode_nav = CALIB;
            printf("disuruh stop Mode\n");
        }
        else if (user_command.empty())
        {
            mode_nav = CALIB;
            printf("Kosong commandnya cok\n");
        }
        else
        {
            printf("SALAH WOI\n");
        }

        flag_command = false;
    }

    if(key_status_[22] == 1) //m
    {
      mode_nav = MANUAL;
      printf("Manual Mode\n");
    }
    else if(key_status_[23] == 1) //n
    {
      mode_nav = CALIB;
      printf("Calib Mode\n");
    }
    else if(key_status_[24] == 1) //o
    {
      mode_nav = SCAN;
      printf("SCAN Mode\n");
    }

    switch (mode_nav)
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
        for (size_t i = 0; i < 8; ++i) {
          if (key_status_[i]) {
          key_flags_[i] = !key_flags_[i];
          }
          gpio_msg.data[i] = key_flags_[i] ? 1 : 0;
          }
        gpio_pub_.publish(gpio_msg);
        break;
      case AFK:
        sendDynamixelCommand(8, "Goal_Velocity", 0);
        sendDynamixelCommand(9, "Goal_Velocity", 0);
        for (int i = 0; i < 8; ++i) {
          gpio_msg_nav.data[i] = 0;
          gpio_msg.data[i] = 0;
        }
        pos_object = 0;
        state_object = 0;
        nav_flag = false;
        nav_flag_msg.data = nav_flag;
        pub_nav_flag.publish(nav_flag_msg);
        gpio_pub_.publish(gpio_msg_nav);
        velflag = false;
        m = 0;
        n = 0;
        break;
      case CALIB:
        if(!velflag)
        {
          servo_mode_msg.data = VELOCITY; // Velocity control mode
          servo_mode_pub_.publish(servo_mode_msg);
          ROS_INFO("Published Velocity Control Mode");
          velflag = true; 
        }
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
            printf("Change to AFK\n");
            if(!nico)
            {
              std_msgs::String msg;
              msg.data = "ready";
              pub_speech.publish(msg);
              printf("ready\n");
              nico = true;
            }
            else
            {
              pubSpeech("done");
              resetSpeechFlag();
            }
            mode_nav = AFK;
            time_counter = 0;
          }
        }
        sendDynamixelCommand(8, "Goal_Velocity", servo_speed[0]);
        sendDynamixelCommand(9, "Goal_Velocity", servo_speed[1]);
        break;
      case SCAN:
        if (class_array.empty() || std::find(class_array.begin(), class_array.end(), user_command) == class_array.end())
        { 
          if (++time_counter > 100)
          {
            if (state_pan == 0)
            {
              sendDynamixelCommand(8, "Goal_Velocity", 10);
              if (position_8_ > 3700)
              {
                printf("scan kanan\n");
                state_pan = 1;
                n++;
              }
            }
            else if (state_pan == 1)
            {
              sendDynamixelCommand(8, "Goal_Velocity", -10);
              if (position_8_ < 1500)
              {
                printf("scan kiri\n");
                n++;
                if(n >= 2)
                {
                  state_pan = 2; 
                  n = 0;
                  sendDynamixelCommand(8, "Goal_Velocity", 0);
                }
                else
                  state_pan = 0;
              }
            }
            else if (state_pan == 2 && m == 0)
            {
              sendDynamixelCommand(9, "Goal_Velocity", 10);
              if (position_9_ > 2304)
              {
                printf("scan bawah\n");
                sendDynamixelCommand(9, "Goal_Velocity", 0);
                state_pan = 0;
                m = 1;
              }
            }
            else if(state_pan == 2 && m == 1)
            {
              sendDynamixelCommand(9, "Goal_Velocity", 10);
              if (position_9_ > 2560)
              {
                printf("scan bawah lagi\n");
                sendDynamixelCommand(9, "Goal_Velocity", 0);
                state_pan = 0;
                m = 2;
              }
            } 
            else if (state_pan == 2 && m == 2)
            {
              sendDynamixelCommand(9, "Goal_Velocity", -10);
              if (position_9_ < 1792)
              {
                printf("scan atas\n");
                sendDynamixelCommand(9, "Goal_Velocity", 0);
                state_pan = 0;
                m = 3;
              }
            }
            else if (state_pan == 2 && m == 3)
            {
              printf("scan selesai object tidak ditemukan\n");
              pubSpeech("nfound " + user_command);
              resetSpeechFlag();
              mode_nav = CALIB;
              m = 0;
              n = 0;
            }
	          for(int i = 0; i < 8; i++)
            {
              gpio_msg_nav.data[i] = 0;
            }
            det = 0;
	          gpio_pub_.publish(gpio_msg_nav);
          }
          else
          {
            sendDynamixelCommand(8, "Goal_Velocity", 0);
            sendDynamixelCommand(9, "Goal_Velocity", 0);
          }
        }
        else
        {
          for (size_t i = 0; i < class_array.size(); ++i)
          {
            if (class_array[i] == user_command)
            {
	            det = 1;
              time_counter = 0;
              xcam = center_x_array[i];
              ycam = center_y_array[i];
              pid_cam(xcam, ycam, 10, 10);

              if (fabs(err_pan) < 10 && fabs(err_tilt) < 10)
              {
                if (position_8_ < 2048 && state_object == 0)
                {
                  printf("object disebelah kanan\n");
                  pubSpeech("right " + user_command);
                  pos_object = RIGHT;
                  state_object = 1;
                }
                else if (position_8_ > 2048 && state_object == 0)
                {
                  printf("object disebelah kiri\n");
                  pubSpeech("left " + user_command);
                  pos_object = LEFT;
                  state_object = 1;
                }
              }
              if (pc_depth < 0.5)
              {
		if(++counter_sampai > 100)
		{
		        //printf("pc_depth: %f\n", pc_depth);
		        if (state_nav_flag == 0 && state_object == 2)
		        {
		          printf("masuk kriteria sampai\n");
		          nav_flag = false;
		          nav_flag_msg.data = nav_flag;
		          pub_nav_flag.publish(nav_flag_msg);
		          state_object = 3;
		          state_nav_flag = 1;
		        }
		        if (state_nav_flag == 1 && state_object == 4)
		        {
		          if (++time_counter_gpio > 150)
		          {
		            printf("beneran sampai ke tujuan hore :)\n");
		            pubSpeech("success " + user_command);
		            resetSpeechFlag();
		            mode_nav = CALIB;
		            time_counter_gpio = 0;
		          }
		          else
		          {
		            if (time_counter_gpio % 25 == 0) // Toggle every 1 second
		            {
		              printf("toggling gpio\n");
		              for (int i = 0; i < 8; ++i)
		              {
		                gpio_msg_nav.data[i] = (gpio_msg_nav.data[i] == 0) ? 1 : 0;
		              }
		            }
		            gpio_pub_.publish(gpio_msg_nav);
		          }
		        }
		}
              }
		else{counter_sampai = 0;}
              sendDynamixelCommand(8, "Goal_Velocity", speed_pan);
              sendDynamixelCommand(9, "Goal_Velocity", speed_tilt);
            }
          }
        }
        break;
    } 
  }
  void timerGpioCallback(const ros::TimerEvent&)
  {
    nav_path_deg = nav_path_deg_buf;

    counter_toggling++;
    if (counter_toggling >= 4)
    {
      toggle_state = !toggle_state;
      counter_toggling = 0;
    }

    if(state_object == 1)
    {
      if(pos_object == LEFT)
      {
        gpio_msg_nav.data[0] = 1; gpio_msg_nav.data[1] = 1; gpio_msg_nav.data[2] = 1; gpio_msg_nav.data[3] = 1;
        gpio_msg_nav.data[4] = 0; gpio_msg_nav.data[5] = 0; gpio_msg_nav.data[6] = 0; gpio_msg_nav.data[7] = 0;
      }
      else if(pos_object == RIGHT)
      {
        gpio_msg_nav.data[0] = 0; gpio_msg_nav.data[1] = 0; gpio_msg_nav.data[2] = 0; gpio_msg_nav.data[3] = 0;
        gpio_msg_nav.data[4] = 1; gpio_msg_nav.data[5] = 1; gpio_msg_nav.data[6] = 1; gpio_msg_nav.data[7] = 1;
      }

      //counter 4 detik timer 4Hz
      if(++time_counter_gpio > 16)
      {
        printf("mulai navigasi\n");
        nav_flag = true;
        nav_flag_msg.data = nav_flag;
        pub_nav_flag.publish(nav_flag_msg);
        resetSpeechFlag();
        state_object = 2;
        time_counter_gpio = 0;
      }
    }
    else if(state_object == 3)
    {
      if(++time_counter_gpio > 16)
      {
        printf("gpio smua mati\n");
        state_object = 4;
        time_counter_gpio = 0;
      }
      else
      { 
        for(int i = 0; i < 8; i++)
        {
          gpio_msg_nav.data[i] = 0;
        }
      }
    }
    else if(state_object == 2 && det == 1)
    {
      if(toggle_state)
      {
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
          // printf("north\n");
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
          // printf("north west\n");
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
          // printf("west\n");
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
          // printf("south west\n");
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
          // printf("south\n");
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
          // printf("south east\n");
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
          // printf("east\n");
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
          // printf("north east\n");
        }
      }
      else
      {
        for(int i = 0; i < 8; i++)
        {
          gpio_msg_nav.data[i] = 0;
        }
      }
    }
    if(mode_nav != MANUAL)
      gpio_pub_.publish(gpio_msg_nav);
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

  void pubSpeech(const std::string speech)
  {
    if(!speech_flag)
    {
      speech_msg.data = speech;
      pub_speech.publish(speech_msg);
      speech_flag = true;
    }
  }

  void resetSpeechFlag()
  {
    speech_flag = false;
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
  ros::Subscriber user_command_sub;
  ros::Publisher pub_nav_flag;
  ros::Publisher pub_speech;
  ros::Subscriber pc_depth_sub;
  ros::Timer timer_;
  ros::Timer timer_gpio;
  std::vector<bool> key_flags_;
  std::vector<uint8_t> key_status_;
  int position_8_ = 0;
  int position_9_ = 0;
  uint8_t mode_nav = CALIB;
  uint8_t operating_mode = 1;
  float target[2], speed[2];
  float servo_speed[2];
  float xcam, ycam;
  std::vector<float> center_x_array, center_y_array;
  std::vector<std::string> class_array;
  double nav_path_deg = 0;
  double nav_path_deg_buf = 0;

  uint32_t time_counter = 0;
  uint32_t time_counter_gpio = 0;
  uint32_t counter_sampai = 0;
  uint32_t counter_tidak_sampai = 0;

  std::string user_command;
  std::string previous_user_command;
  // object_nav mode_nav = CALIB;

  uint8_t pos_object = 0;
  uint8_t state_object = 0;
  uint8_t state_pan = 0;  
  // uint8_t state_command = 0;
  uint8_t state_nav_flag = 0;

  bool nav_flag = false;
  bool flag_command = false;
  bool velflag = false;
  bool speech_flag = false;
  bool nico = false;

  bool toggle_state = false;
  uint8_t counter_toggling = 0;

  uint8_t det = 0;
  
  std_msgs::Bool nav_flag_msg;
  std_msgs::UInt8 servo_mode_msg;
  std_msgs::String speech_msg;

  float pc_depth;

  /*Book
Bottle
Chair
Dispenser
Door
Glass
Laptop
Person
Table*/
  std::string object_targets[9] = {"Book", "Bottle", "Chair", "Dispenser", "Door", "Glass", "Laptop", "Person", "Table"};
  std_msgs::Int32MultiArray gpio_msg_nav;
  std_msgs::Int32MultiArray gpio_msg;

  uint8_t n = 0, m = 0;

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