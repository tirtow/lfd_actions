#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <list>
#include <map>
#include <termios.h>
#include "sensor_msgs/JointState.h"

using std::string;
using std::cin;
using std::cout;
using std::ofstream;
using std::vector;
using std::list;
using std::map;
using std::multimap;

// struct for the different values obtained from the joints
struct joint_state {
    double vel;
    double pos;
    double eff;
};

// The topic the arm joints publish to
const string ARM_TOPIC = "/joint_states";

// lists of joint_states
list<joint_state> joint_1;
list<joint_state> joint_2;
list<joint_state> joint_3;
list<joint_state> joint_4;
list<joint_state> joint_5;
list<joint_state> joint_6;
list<joint_state> finger_1;
list<joint_state> finger_2;

// lists of temporal bins for the joint_states
list<joint_state> joint_1_temp;
list<joint_state> joint_2_temp;
list<joint_state> joint_3_temp;
list<joint_state> joint_4_temp;
list<joint_state> joint_5_temp;
list<joint_state> joint_6_temp;
list<joint_state> finger_1_temp;
list<joint_state> finger_2_temp;

/**
 * Reads a character without blocking execution
 * Returns the read character
 */
int getch() {
    static struct termios oldt, newt;

    // Saving old settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Disable buffering
    newt.c_lflag &= ~(ICANON);
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 0;

    // Apply new settings
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // Read character without blocking
    int c = getchar();

    // Restore old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return c;
}

/**
 * Records the recieved data into the map of joint names to the joint states
 */
void arm_cb(const sensor_msgs::JointState::ConstPtr& msg) {
    vector<string> names = msg->name;
    if (names.size() == 8) {
        vector<double> positions = msg->position;
        vector<double> velocities = msg->velocity;
        vector<double> efforts = msg->effort;

        for (int i = 0; i < names.size(); i++) {
            joint_state state;
            state.pos = positions[i];
            state.vel = velocities[i];
            state.vel = efforts[i];

            switch (i) {
            case 0:
                joint_1.push_back(state);
                break;
            case 1:
                joint_2.push_back(state);
                break;
            case 2:
                joint_3.push_back(state);
                break;
            case 3:
                joint_4.push_back(state);
                break;
            case 4:
                joint_5.push_back(state);
                break;
            case 5:
                joint_6.push_back(state);
                break;
            case 6:
                finger_1.push_back(state);
                break;
            case 7:
                finger_2.push_back(state);
                break;
            }
        }
    }
}

/**
 * Splits the raw_data into temporal bins into the output
 */
void split(list<joint_state> raw_data, list<joint_state> output) {
    list<joint_state>::size_type size = raw_data.size();
    int step = std::ceil(size / 10.0);

    for (list<joint_state>::size_type start = 0; start < raw_data.size(); start += step) {
        double pos_sum = 0;
        double vel_sum = 0;
        double eff_sum = 0;

        list<joint_state>::const_iterator it = raw_data.begin();
        for (int i = 0; i < start; i++) {
            it++;
        }

        list<joint_state>::const_iterator stop = it;
        for (int i = 0; i < step; i++) {
            stop++;
        }

        while (it != stop && it != raw_data.end()) {
            pos_sum += it->pos;
            vel_sum += it->vel;
            eff_sum += it->eff;
            it++;
        }

        joint_state result;
        result.pos = pos_sum / step;
        result.vel = vel_sum / step;
        result.eff = eff_sum / step;

        output.push_back(result);
    }
}

/**
 * Prints a list of joint_states to the specified ofstreams
 */
void print_list(list<joint_state> l, ofstream& pos_out, ofstream& vel_out,
        ofstream& eff_out) {
    list<joint_state>::const_iterator it = l.begin();

    if (it != l.end()) {
        pos_out << it->pos;
        vel_out << it->vel;
        eff_out << it->eff;
        it++;
    }

    for (; it != l.end(); it++) {
        pos_out << ", " << it->pos;
        vel_out << ", " << it->vel;
        eff_out << ", " << it->eff;
    }

    pos_out << "\n";
    vel_out << "\n";
    eff_out << "\n";
}

int main(int argc, char** argv) {
    // Initializing the ros node
    ros::init(argc, argv, "arff_recorder");
    ros::NodeHandle n;

    // Creating the subscriber
    ros::Subscriber arm_sub = n.subscribe(ARM_TOPIC, 1000, arm_cb);

    // Setting the filenames
    string dir = "";
    string pos_file = "positions.txt";
    string vel_file = "velocities.txt";
    string eff_file = "efforts.txt";

    // Waiting to record
    string start;
    cout << "Enter 1 to start: ";
    cin >> start;
    
    // Recording the data
    while (ros::ok() && getch() != 'q') {
        ros::spinOnce();
    }

    // Dumping data
    ofstream pos_out;
    ofstream vel_out;
    ofstream eff_out;
    pos_out.open((dir + pos_file).c_str());
    vel_out.open((dir + vel_file).c_str());
    eff_out.open((dir + eff_file).c_str());

    // Put into temporal bins
    split(joint_1, joint_1_temp);
    split(joint_2, joint_2_temp);
    split(joint_3, joint_3_temp);
    split(joint_4, joint_4_temp);
    split(joint_5, joint_5_temp);
    split(joint_6, joint_6_temp);
    split(finger_1, finger_1_temp);
    split(finger_2, finger_2_temp);

    print_list(joint_1_temp, pos_out, vel_out, eff_out);
    print_list(joint_2_temp, pos_out, vel_out, eff_out);
    print_list(joint_3_temp, pos_out, vel_out, eff_out);
    print_list(joint_4_temp, pos_out, vel_out, eff_out);
    print_list(joint_5_temp, pos_out, vel_out, eff_out);
    print_list(joint_6_temp, pos_out, vel_out, eff_out);
    print_list(finger_1_temp, pos_out, vel_out, eff_out);
    print_list(finger_2_temp, pos_out, vel_out, eff_out);

    pos_out.close();
    vel_out.close();
    eff_out.close();

    return 0;
}
