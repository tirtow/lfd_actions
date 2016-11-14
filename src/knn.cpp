#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <termios.h>
#include "dataset.h"
#include "dataset.cpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "action.h"
#include "action.cpp"

#define NUM_JOINTS 8
#define NUM_BINS 10

using std::string;
using std::cin;
using std::cout;
using std::endl;
using std::istream;
using std::vector;
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using sensor_msgs::JointState;

// The topic the arm joints publish to
const string ARM_TOPIC = "/joint_states";
const string CART_TOPIC = "/mico_arm_driver/out/tool_position";

// Vectors used to record the action in callback
vector<Pose> poses;
Action::joint_list joints;

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
 * Splits the joints into the temporal bins
 * Sums up the velocities, positions, and efforts and divides by the
 * number of bins
 * Returns a joint_list that contains the bins for each joint
 */
/*
Dataset::joint_list split_bins() {
    Dataset::joint_list bins;
    int bin_length = (joints.size() / NUM_JOINTS) / NUM_BINS;

    // Looping through the number of bins
    for (int bin = 0; bin < NUM_BINS; bin++) {
        Dataset::joint_list sums;

        // Getting an iterator to the start of the bin
        int start = bin * bin_length;
        Dataset::joint_cit it = joints.begin();
        for (int i = 0; i < start; i++) {
            it++;
        }

        // Summing up the velocity, position, and effort for each joint
        for (int raw_index = bin * bin_length; raw_index < bin_length; raw_index++) {
            for (int sum_index = 0; sum_index < NUM_JOINTS; sum_index++) {
                sums[sum_index].vel += joints[raw_index].vel;
                sums[sum_index].pos += joints[raw_index].pos;
                sums[sum_index].eff += joints[raw_index].eff;
            }
        }

        // Getting the averages and pushing
        for (int i = 0; i < NUM_JOINTS; i++) {
            sums[i].vel /= NUM_BINS;
            sums[i].pos /= NUM_BINS;
            sums[i].eff /= NUM_BINS;

            bins.push_back(sums[i]);
        }
    }

    return bins;
}
*/

/**
 * Builds the temporal bin specified by the bin_num
 * Gets the pose for the bin and the joint_states for the bin
 * Returns a bin with the appropriate pose and joint_states
 */
/*
Dataset::bin build_bin(int bin_num, const Dataset::joint_list& joint_bins) {
    Dataset::bin bin;

    // Getting the pose
    bin.pose = cartesian[bin_num];

    // Getting the joint_set_list
    int offset = bin_num * NUM_BINS;
    for (int i = 0; i < NUM_JOINTS; i++) {
        bin.joints.push_back(joints[i + offset]);
    }

    return bin;
}
*/

/**
 * Builds the action from the joint_bins
 * Sets the label to an empty string as the label for the action has not
 * been determined
 * Returns the new action
 */
/*
Dataset::action build_action(const Dataset::joint_list& joint_bins) {
    Dataset::action ac;
    ac.label = "";

    // Building the bins
    for (int bin = 0; bin < NUM_BINS; bin++) {
        ac.data.push_back(build_bin(bin, joint_bins));
    }

    return ac;
}
*/

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
    ROS_ERROR("Usage: rosrun lfd_actions knn -src <dataset file>"
              "\n\t\t\t\t(Optional)\n\t\t\t\t -super <true/false>"
              "\n\t\t\t\t-p");
}

void callback(const JointState::ConstPtr& joint, const PoseStamped::ConstPtr& cart) {
    // Pushing the joint states
    vector<string> names = joint->name;
    // Checking that received the appropriate number of joints
    if (names.size() == 8) {
        // Getting the positions, velocities, and efforts
        vector<double> positions = joint->position;
        vector<double> velocities = joint->velocity;
        vector<double> efforts = joint->effort;

        // Looping through each joint
        for (int i = 0; i < names.size(); i++) {
            // Creating a joint_state for the joint
            Action::joint_state state;
            state.pos = positions[i];
            state.vel = velocities[i];
            state.vel = efforts[i];

            // Pushing to the joints list
            joints.push_back(state);
        }
    }

    // Pushing the cartesian pose
    poses.push_back(cart->pose);
}

int main(int argc, char** argv) {
    // Initializing the ros node
    ros::init(argc, argv, "arff_recorder");
    ros::NodeHandle n;

    // Creating the subscribers
    message_filters::Subscriber<JointState> arm_sub(n, ARM_TOPIC, 1000);
    message_filters::Subscriber<PoseStamped> cart_sub(n, CART_TOPIC, 1000);
    message_filters::TimeSynchronizer<JointState, PoseStamped> sync(
            arm_sub, cart_sub, 1000);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // Getting command line arguments
    string dataset_name;
    bool supervised = false;
    bool found_d = false;
    bool found_p = false;
    for (int i = 1; i < argc; i++) {
        string argv_str(argv[i]);
        found_p = argv_str == "-p";
        if (argv_str == "-src" || argv_str == "--source") {
            if (i + 1 <= argc) {
                dataset_name = argv[++i];
                found_d = true;
            } else {
                print_err();
                return 1;
            }
        } else if (argv_str == "-super" || argv_str == "--supervise") {
            if (i + 1 <= argc) {
                supervised = string(argv[++i]) == "true";
            } else {
                print_err();
                return 1;
            }
        }
    }

    // Checking that a dataset file has been specified
    if (!found_d) {
        print_err();
        return 1;
    }

    // Outputing command line args
    ROS_INFO("dataset path: %s", dataset_name.c_str());
    if (supervised) {
        ROS_INFO("supervised = true");
    } else {
        ROS_INFO("supervised = false");
    }

    // Building the dataset
    Dataset dataset(dataset_name, 3);

    // Printing the dataset if -p in commandline args
    if (found_p) {
        //dataset.print_dataset();
    }

    bool again = true;
    while (again) {
        // Clearing the vectors
        poses.clear();
        joints.clear();

        // Waiting to record
        cout << "Press [Enter] to start";
        cin.ignore();

        // Notifying of recording
        cout << "Recording data..." << endl
             << "Press \'q\' to stop" << endl;

        // Recording the data
        while (ros::ok() && getch() != 'q') {
            ros::spinOnce();
        }
        cout << endl;

        // Creating the recorded action
        Action ac(poses, joints);

        // Guessing the classification
        string guess = dataset.guess_classification(ac);

        // Print out the guess for the action
        print_guess(guess);

        // If supervised checking guess with user
        if (supervised) {
            ac.set_label(confirm_guess(guess));
            dataset.update(ac);
        }

        // Getting whether or not to record another action
        again = repeat();
    }

    return 0;
}
