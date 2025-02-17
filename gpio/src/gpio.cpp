#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <vector>

void writeToFile(const std::string& path, const std::string& value) {
    std::ofstream file(path);
    if (file.is_open()) {
        file << value;
        file.close();
    } else {
        std::cerr << "Unable to open file: " << path << std::endl;
    }
}

void exportGPIO(int gpio) {
    writeToFile("/sys/class/gpio/export", std::to_string(gpio));
}

void setDirection(int gpio, const std::string& direction) {
    writeToFile("/sys/class/gpio/gpio" + std::to_string(gpio) + "/direction", direction);
}

void writeValue(int gpio, int value) {
    writeToFile("/sys/class/gpio/gpio" + std::to_string(gpio) + "/value", std::to_string(value));
}

int readValue(int gpio) {
    std::ifstream file("/sys/class/gpio/gpio" + std::to_string(gpio) + "/value");
    int value = -1;
    if (file.is_open()) {
        file >> value;
        file.close();
    } else {
        std::cerr << "Unable to open file: " << "/sys/class/gpio/gpio" + std::to_string(gpio) + "/value" << std::endl;
    }
    return value;
}

class GPIOControl {
public:
    GPIOControl() {
        gpios_ = {421, 422, 393, 448, 482, 429, 447, 446};
        prev_values_ = std::vector<int>(8, -1); // Initialize with -1 to ensure the first write happens

        // Export GPIOs and set direction to output
        for (int gpio : gpios_) {
            exportGPIO(gpio);
            usleep(100000); // Sleep for 100ms to allow the export to complete
            setDirection(gpio, "out");
        }
    }

    void gpioCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
        if (msg->data.size() != 8) {
            ROS_ERROR("Invalid input size. Please provide an array of size 8.");
            return;
        }

        // Set GPIO values based on input array
        for (size_t i = 0; i < 8; ++i) {
            if (msg->data[i] != prev_values_[i]) {
                writeValue(gpios_[i], msg->data[i]);
                usleep(100000); // Sleep for 100ms to allow the write to complete
                int written_value = readValue(gpios_[i]);
                if (written_value == msg->data[i]) {
                    ROS_INFO("GPIO %d written with value %d", gpios_[i], msg->data[i]);
                } else {
                    ROS_ERROR("Failed to write GPIO %d with value %d", gpios_[i], msg->data[i]);
                }
                prev_values_[i] = msg->data[i];
            }
        }
    }

private:
    std::vector<int> gpios_;
    std::vector<int> prev_values_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gpio_control_node");
    ros::NodeHandle nh;

    GPIOControl gpio_control;

    ros::Subscriber sub = nh.subscribe("gpio_control", 10, &GPIOControl::gpioCallback, &gpio_control);

    ros::spin();

    return 0;
}