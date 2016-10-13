#include <ros/ros.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include <list>
#include <vector>
#include <map>
#include <termios.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

using std::cin;
using std::cout;
using std::endl;
using std::ofstream;
using std::string;
using std::list;
using std::map;
using std::vector;

// Structure for the joint state for a joint at a time step
struct joint_state {
    double pos;
    double vel;
    double eff;
};

// Constants for the recorder
const string ARM_TOPIC = "/joint_states";
const string DEFAULT_FILENAME = "actions_data.arff";
//const string CLASSIFICATIONS = {"alpha", "bravo"};
const int NEW_ACTION = 1;

// Variables to record data
int longest = 0;
list<double> action;
list<list<double> > info;
list<string> classifications;
map<string, list<joint_state> > data;

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
 * Gets the choice to record a new action or to write and quit
 * Returns the int for the choice
 */
int get_choice() {
    cout << endl << NEW_ACTION << ".  Record new action" << endl
         << "2.  Quit" << endl
         << "Choice: ";

    int choice;
    cin >> choice;

    return choice;
}

/**
 * Gets the classification for the action from the user
 * Returns the classification entered by the user
 */
string get_classification() {
    cout << "Enter action classification (lift, grasp, punch, swipe, press): ";
    string result;
    cin >> result;

    return result;
}

/**
 * Writes the recorded data to the ofstream
 */
void write_data(ofstream& output) {
    output << "@relation actions\n";

    // Writing the attributes
    for (int t = 0; t < longest; t++) {
        for (int joint = 1; joint <= 6; joint++) {
            output << "@attribute \'t" << t << "-joint" << joint << "\' numeric\n";
        }
    }

    // Writing the classes
    output << "@attribute \'Class\' { \'lift\', \'grasp\', '\'punch\', \'swipe\', \'press\'}\n";

    // Writing the data
    output << "@data\n";

    // Printing the lists
    list<list<double> >::const_iterator info_it = info.begin();
    list<string>::const_iterator class_it = classifications.begin();

    while (info_it != info.end()) {
        list<double> temp = *info_it++;

        // Writing the values
        list<double>::const_iterator it = temp.begin();
        while (it != temp.end()) {
            output << *it++ << ",";
        }

        // Writing the padding
        for (int i = temp.size(); i < longest * 6; i++) {
            output << "0,";
        }

        // Write the classification
        output << "\'" << *class_it++ << "\'" << "\n";
    }
}

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
            state.eff = efforts[i];

            data[names[i]].push_back(state);
        }
    }
}

int main(int argc, char** argv) {
    // Initializing the ros node
    ros::init(argc, argv, "data_recorder");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // Creating the subscribers
    ros::Subscriber arm_sub = n.subscribe(ARM_TOPIC, 1000, arm_cb);

    // Getting the filename
    string filename;
    if (argc > 1) {
        filename = argv[1];
    } else {
        filename = DEFAULT_FILENAME;
    }

    // Opening the stream
    ofstream output;
    output.open(filename.c_str());

    // Looping to get the data
    int choice = get_choice();
    while (choice == 1) {
        classifications.push_back(get_classification());
        ROS_INFO("Recording action...");

        int iterations = 0;
        while (ros::ok() && getch() != 'q') {
            iterations++;
            ros::spinOnce();
            loop_rate.sleep();
        }

        // Adding the recorded data to the double list and clearing
        ROS_INFO("Storing recorded data...");
        info.push_back(action);
        action.clear();

        // Update the longest data set
        longest = std::max(longest, iterations);

        // Getting whether to continue
        choice = get_choice();
    }

    // Writing the recorded data
    ROS_INFO("Writing to %s...", filename.c_str());
    write_data(output);

    // Done writing, closing the stream
    output.close();

    return 0;
}
