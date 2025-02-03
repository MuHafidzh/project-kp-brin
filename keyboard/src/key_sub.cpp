#include <ros/ros.h>
#include <keyboard/keyb.h>
#include <vector>
#include <unordered_map>
#include <iostream>

void keyCallback(const keyboard::keyb::ConstPtr& msg) {
    std::vector<uint8_t> key_status = msg->keys;
    std::unordered_map<size_t, char> index_map = {
        {0, '0'}, {1, '1'}, {2, '2'}, {3, '3'}, {4, '4'}, {5, '5'}, {6, '6'}, {7, '7'}, {8, '8'}, {9, '9'},
        {10, 'a'}, {11, 'b'}, {12, 'c'}, {13, 'd'}, {14, 'e'}, {15, 'f'}, {16, 'g'}, {17, 'h'}, {18, 'i'}, {19, 'j'},
        {20, 'k'}, {21, 'l'}, {22, 'm'}, {23, 'n'}, {24, 'o'}, {25, 'p'}, {26, 'q'}, {27, 'r'}, {28, 's'}, {29, 't'},
        {30, 'u'}, {31, 'v'}, {32, 'w'}, {33, 'x'}, {34, 'y'}, {35, 'z'}, {36, '+'}, {37, '-'},
        {38, 'U'}, {39, 'D'}, {40, 'R'}, {41, 'L'} // Up, Down, Right, Left arrows
    };

    // std::cout << "Keys pressed: ";
    // for (size_t i = 0; i < key_status.size(); ++i) {
    //     if (key_status[i]) {
    //         std::cout << index_map[i] << " ";
    //     }
    // }
    // std::cout << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "key_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber key_sub = nh.subscribe("key", 10, keyCallback);

    ros::spin();

    return 0;
}