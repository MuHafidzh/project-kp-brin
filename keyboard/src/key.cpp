#include <ros/ros.h>
#include <keyboard/keyb.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <vector>
#include <unordered_map>

int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "key_node");
    ros::NodeHandle nh;
    ros::Publisher key_pub = nh.advertise<keyboard::keyb>("key", 100);

    std::vector<uint8_t> key_status(42, 0); // Initialize array with size 42 and all elements set to 0

    // Map to store the index of each key
    std::unordered_map<char, size_t> key_map = {
        {'0', 0}, {'1', 1}, {'2', 2}, {'3', 3}, {'4', 4}, {'5', 5}, {'6', 6}, {'7', 7}, {'8', 8}, {'9', 9},
        {'a', 10}, {'b', 11}, {'c', 12}, {'d', 13}, {'e', 14}, {'f', 15}, {'g', 16}, {'h', 17}, {'i', 18}, {'j', 19},
        {'k', 20}, {'l', 21}, {'m', 22}, {'n', 23}, {'o', 24}, {'p', 25}, {'q', 26}, {'r', 27}, {'s', 28}, {'t', 29},
        {'u', 30}, {'v', 31}, {'w', 32}, {'x', 33}, {'y', 34}, {'z', 35}, {'+', 36}, {'-', 37},
        {'U', 38}, {'D', 39}, {'R', 40}, {'L', 41} // Up, Down, Right, Left arrows
    };

    ros::Rate loop_rate(50);
    while (ros::ok()) {
        // Reset all key statuses to 0
        std::fill(key_status.begin(), key_status.end(), 0);

        if (kbhit()) {
            uint8_t c = getchar();
            if (c == '\033') { // handle escape sequences
                getchar(); // skip the [
                switch(getchar()) {
                    case 'A': c = 'U'; break; // up arrow
                    case 'B': c = 'D'; break; // down arrow
                    case 'C': c = 'R'; break; // right arrow
                    case 'D': c = 'L'; break; // left arrow
                }
            }

            if (key_map.find(c) != key_map.end()) { // only consider specified keys
                size_t index = key_map[c];
                key_status[index] = 1; // Set the status of the key to 1
            }
        }

        keyboard::keyb msg;
        msg.keys = key_status;
        key_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}