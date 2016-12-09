#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <string>
#include <termios.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include "action.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using std::cin;
using std::cout;
using std::endl;
using std::ofstream;
using std::list;
using std::vector;
using std::string;
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using sensor_msgs::JointState;

// The topic the arm joints publish to
const string ARM_TOPIC = "/joint_states";
const string CART_TOPIC = "/mico_arm_driver/out/tool_position";

// lists used to record the action in callback
list<Pose> poses;
list<JointState> joints;
list<ros::Time> times;

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
 * Prompts the user to record another actions
 * Returns true if going to record another action, false otherwise
 */
bool repeat() {
    string repeat, clear;
    cout << "Again [Y/y]: ";
    cin >> repeat;
    getline(cin, clear);

    return repeat == "Y" || repeat == "y";
}

void callback(const JointState::ConstPtr& joint, const PoseStamped::ConstPtr& cart) {
    // Getting the names
    vector<string> names = joint->name;

    // Checking that received the appropriate number of joints
    if (names.size() == 8) {
        // Pushing the values
        joints.push_back(*joint);
        poses.push_back(cart->pose);
        times.push_back(cart->header.stamp);
    }
}

/**
 * Performs the classification by recording the actions from ros subscribers
 */
void record(ofstream& os) {
    ros::Rate loop_rate(20);
    bool again = true;
    while (again) {
        // Clearing the vectors
        poses.clear();
        joints.clear();
        times.clear();

        // Waiting to record
        cout << "Press [Enter] to start";
        cin.ignore();

        // Notifying of recording
        cout << "Recording data..." << endl
             << "Press \'q\' to stop" << endl;

        // Recording the data
        while (ros::ok() && getch() != 'q') {
            ros::spinOnce();
            loop_rate.sleep();
        }
        cout << endl;

        string label;
        cout << "Enter label: ";
        cin >> label;

        // Creating and writing the recorded action
        Action ac(poses, joints, times);
        ac.set_label(label);
        ac.print(os);
        
        // Getting whether to repeat
        again = repeat();
    }
}

void print_err() {
    cout << "recorder: Missing required command line argument" << endl
         << "Usage: rosrun lfd_actions recorder -d <file> [-h]" << endl
         << "Options:" << endl
         << "  -h           Print this help message." << endl
         << "  -d <file>    The file to record to." << endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "recorder");
    ros::NodeHandle n;

    // Creating the subscribers
    message_filters::Subscriber<JointState> arm_sub(n, ARM_TOPIC, 100);
    message_filters::Subscriber<PoseStamped> cart_sub(n, CART_TOPIC, 100);
    typedef message_filters::sync_policies::ApproximateTime<
            JointState, PoseStamped> policy;
    message_filters::Synchronizer<policy> sync(policy(10), arm_sub, cart_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // Getting the command line args
    string file = "";
    for (int i = 1; i < argc; i++) {
		string argv_str(argv[i]);
        if (argv_str == "-h") {
            print_err();
            return 1;
        } else if (argv_str == "-d") {
            if (i + 1 < argc) {
				ROS_INFO("1");
                file = argv[++i];
            } else {
                print_err();
                return 1;
            }
        }
    }
    
    ROS_INFO("file: %s", file.c_str());

    // Checking for file
    if (file == "") {
        print_err();
        return 1;
    }

    // Getting the file
    ofstream os;
    os.open(file.c_str(), std::ios_base::app);

    // Recording actions
    record(os);
    
    return 0;
}
