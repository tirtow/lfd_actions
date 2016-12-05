#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <list>
#include <termios.h>
#include "dataset.h"
#include "dataset.cpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Header.h>
#include "action.h"
#include "dtw.h"

#define NUM_JOINTS 8
#define DEFAULT_K 3
#define THRESHOLD 0.05

using std::string;
using std::cin;
using std::cout;
using std::endl;
using std::istream;
using std::ifstream;
using std::vector;
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using sensor_msgs::JointState;
using std::list;

// The topic the arm joints publish to
const string ARM_TOPIC = "/joint_states";
const string CART_TOPIC = "/mico_arm_driver/out/tool_position";

// lists used to record the action in callback
list<Pose> poses;
list<JointState> joints;
list<ros::Time> times;

// Values to compare previous
JointState prev_jointstate;
Pose prev_pose;

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

/**
 * Outputs the guess from the robot
 */
void print_guess(const string& guess) {
    ROS_INFO("Action guess: %s", guess.c_str());
}

/**
 * Prompts the user to correct the robot if the robot guessed wrong
 * Returns the appropriate label for the action that was performed
 */
string confirm_guess(const string& guess) {
    string confirm;
    string label = guess;

    // Getting if guess correct
    cout << "Guess correct? [Y/N]: ";
    cin >> confirm;

    // If incorrect, getting correct label
    if (confirm != "Y" && confirm != "y") {
        cout << "Enter the correct label: ";
        cin >> label;
    }

    return label;
}

/**
 * Displays error when incorrect command line args given
 */
void print_err() {
    cout << "knn: Missing required command line argument" << endl
         << "Usage: rosrun lfd_actions knn -d <file> [-hv] [-k <int>]"
         << " [-t <file>]" << endl
         << "Options:" << endl
         << "  -h           Print this help message." << endl
         << "  -v           Optional verbose flag." << endl
         << "  -s           Optional supervise flag." << endl
         << "  -d <file>    The dataset file." << endl
         << "  -t <file>    The test file." << endl
         << "  -k <int>     The number of nearest neighbors" << endl;
}

double get_joint_diff(const JointState new_jointstate) {
    double diff = 0;
    for (int i = 0; i < NUM_JOINTS; i++) {
        diff += new_jointstate.position[i] - prev_jointstate.position[i];
        diff += new_jointstate.velocity[i] - prev_jointstate.velocity[i];
        diff += new_jointstate.effort[i] - prev_jointstate.effort[i];
    }

    return diff;
}

double get_pose_diff(const Pose new_pose) {
    double diff = 0;
    
    diff += new_pose.position.x - prev_pose.position.x;
    diff += new_pose.position.y - prev_pose.position.y;
    diff += new_pose.position.z - prev_pose.position.z;

    diff += new_pose.orientation.x - prev_pose.orientation.x;
    diff += new_pose.orientation.y - prev_pose.orientation.y;
    diff += new_pose.orientation.z - prev_pose.orientation.z;
    diff += new_pose.orientation.w - prev_pose.orientation.w;

    return diff;
}

double get_difference(const JointState new_jointstate, const Pose new_pose) {
    return abs(get_joint_diff(new_jointstate) + get_pose_diff(new_pose));
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

        // Getting the difference
        double diff = get_difference(*joint, cart->pose);
        if (diff < THRESHOLD) {
            ROS_INFO("Difference: %f", diff);
        }
    }
}

void perform_classification(Dataset& dataset, Action& ac, Action& ac_offset,
        bool verbose, bool supervise) {
    // Guessing the classification
    string guess = dataset.guess_classification(ac_offset, verbose);

    // Print out the guess for the action
    print_guess(guess);

    // If supervised checking guess with user
    if (supervise) {
        ac.set_label(confirm_guess(guess));
        dataset.update(ac);
    }
}

/**
 * Performs the classification by recording the actions from ros subscribers
 */
void record(Dataset& dataset, bool verbose, bool supervise) {
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

        // Creating the recorded action
        Action ac(poses, joints, times);
        Action ac_offset(poses, joints, times);
        
        // Performing the classification
        perform_classification(dataset, ac, ac_offset, verbose, supervise);

        again = repeat();
    }
}

void test_file(Dataset& dataset, const string& file, bool verbose, bool supervise) {
    // Opening the test file
    ifstream is(file.c_str());

    // Looping through all the actions
    while (is) {
        string line;
        getline(is, line);
        
        if (line != "") {
            // Getting the action
            Action ac(line);
            Action ac_offset(line);

            // Performing the classification
            perform_classification(dataset, ac, ac_offset, verbose, supervise);
            cout << endl;
        }
    }
}

int main(int argc, char** argv) {
    // Initializing the ros node
    ros::init(argc, argv, "knn");
    ros::NodeHandle n;

    // Getting command line arguments
    string dataset_name = "";;
    string testfile_name = "";
    bool supervise = false;
    bool verbose = false;
    int k = DEFAULT_K;
    for (int i = 1; i < argc; i++) {
        string argv_str(argv[i]);

        if (argv_str == "-h") {
            // Print help message and stop
            print_err();
            return 1;
        } else if (argv_str == "-d") {
            // Getting the dataset
            if (i + 1 <= argc) {
                dataset_name = argv[++i];
            } else {
                print_err();
                return 1;
            }
        } else if (argv_str == "-t") {
            // Getting the test file
            if (i + 1 <= argc) {
                testfile_name = argv[++i];
            } else {
                print_err();
                return 1;
            }
        } else if (argv_str == "-k") {
            // Setting the number of nearest neighbors
            if (i + 1 <= argc) {
                k = atoi(argv[++i]);
            } else {
                print_err();
                return 1;
            }
        } else {
            // Other flags
            verbose = verbose || argv_str == "-v";
            supervise = supervise || argv_str == "-s";
        }
    }

    // Checking that a dataset file has been specified
    if (dataset_name == "") {
        print_err();
        return 1;
    }

    // Outputing command line args
    ROS_INFO("dataset path: %s", dataset_name.c_str());
    if (supervise) {
        ROS_INFO("supervised = true");
    } else {
        ROS_INFO("supervised = false");
    }

    // Building the dataset
    Dataset dataset(dataset_name, verbose, k);

    // Checking whether running a test file or recording actions
    if (testfile_name != "") {
        test_file(dataset, testfile_name, verbose, supervise);
    } else {
        // Creating the subscribers
        message_filters::Subscriber<JointState> arm_sub(n, ARM_TOPIC, 100);
        message_filters::Subscriber<PoseStamped> cart_sub(n, CART_TOPIC, 100);
        typedef message_filters::sync_policies::ApproximateTime<
                JointState, PoseStamped> policy;
        message_filters::Synchronizer<policy> sync(policy(10), arm_sub, cart_sub);
        sync.registerCallback(boost::bind(&callback, _1, _2));

        // Recording actions
        record(dataset, verbose, supervise);
    }

    return 0;
}
