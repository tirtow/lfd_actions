#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <list>
#include <termios.h>
#include <cmath>
#include <climits>
#include "sensor_msgs/JointState.h"
#include "dataset.h"
#include "dataset.cpp" 
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

using std::string;
using std::cin;
using std::cout;
using std::endl;
using std::istream;
using std::ifstream;
using std::ofstream;
using std::vector;
using std::list;
using geometry_msgs::Pose;

// The topic the arm joints publish to
const string ARM_TOPIC = "/joint_states";
const string CART_TOPIC = "/mico_arm_driver/out/tool_position";

list<geometry_msgs::Pose> cartesian;
list<geometry_msgs::Pose> cartesian_temp;

// lists of joint_states
Dataset::action_list joint_1;
Dataset::action_list joint_2;
Dataset::action_list joint_3;
Dataset::action_list joint_4;
Dataset::action_list joint_5;
Dataset::action_list joint_6;
Dataset::action_list finger_1;
Dataset::action_list finger_2;

// lists of temporal bins for the coord
Dataset::action_list joint_1_temp;
Dataset::action_list joint_2_temp;
Dataset::action_list joint_3_temp;
Dataset::action_list joint_4_temp;
Dataset::action_list joint_5_temp;
Dataset::action_list joint_6_temp;
Dataset::action_list finger_1_temp;
Dataset::action_list finger_2_temp;

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

    // Checking that received the appropriate number of joints
    if (names.size() == 8) {
        // Getting the positions, velocities, and efforts
        vector<double> positions = msg->position;
        vector<double> velocities = msg->velocity;
        vector<double> efforts = msg->effort;

        // Looping through each joint
        for (int i = 0; i < names.size(); i++) {
            // Creating a coord for the joint
            Dataset::data_point state;
            state.pos = positions[i];
            state.vel = velocities[i];
            state.vel = efforts[i];

            // Pushing the coord to the appropriate list
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

void cart_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    cartesian.push_back(msg->pose);
}

/**
 * Splits the raw_data into temporal bins into the output
 */
void split(Dataset::action_list& raw_data, Dataset::action_list& output) {
    Dataset::action_list::size_type size = raw_data.size();
    int step = std::ceil(size / 10.0);

    // Splitting the list into 10 sets and looping through each set
    for (Dataset::action_list::size_type start = 0; start < raw_data.size();
            start += step) {
        double pos_sum = 0;
        double vel_sum = 0;
        double eff_sum = 0;

        // Moving it to the beginning of the set
        Dataset::data_list_cit it = raw_data.begin();
        for (int i = 0; i < start; i++) {
            it++;
        }

        // Moving stop to the end of the set
        Dataset::data_list_cit stop = it;
        for (int i = 0; i < step; i++) {
            stop++;
        }

        // Summing the positions, velocities, and efforts for the set
        while (it != stop && it != raw_data.end()) {
            pos_sum += it->pos;
            vel_sum += it->vel;
            eff_sum += it->eff;
            it++;
        }

        // Creating the coord for the temporal bin and pushing to the result list
        Dataset::data_point result;
        result.pos = pos_sum / step;
        result.vel = vel_sum / step;
        result.eff = eff_sum / step;
        output.push_back(result);
    }
}

/**
 * Splits the raw_data into temporal bins into the output
 */
void split(list<Pose>& raw_data, list<Pose>& output) {
    Dataset::action_list::size_type size = raw_data.size();
    int step = std::ceil(size / 10.0);

    // Splitting the list into 10 sets and looping through each set
    for (Dataset::action_list::size_type start = 0; start < raw_data.size();
            start += step) {
        double pos_x = 0;
        double pos_y = 0;
        double pos_z = 0;
        double ori_x = 0;
        double ori_y = 0;
        double ori_z = 0;
        double ori_w = 0;

        // Moving it to the beginning of the set
        list<Pose>::const_iterator it = raw_data.begin();
        for (int i = 0; i < start; i++) {
            it++;
        }

        // Moving stop to the end of the set
        list<Pose>::const_iterator stop = it;
        for (int i = 0; i < step; i++) {
            stop++;
        }

        // Summing the positions, velocities, and efforts for the set
        while (it != stop && it != raw_data.end()) {
            pos_x += it->position.x;
            pos_y += it->position.y;
            pos_z += it->position.z;
            ori_x += it->orientation.z;
            ori_y += it->orientation.z;
            ori_z += it->orientation.z;
            ori_w += it->orientation.z;
            it++;
        }

        Pose result;
        result.position.x = pos_x / step;
        result.position.y = pos_y / step;
        result.position.z = pos_z / step;
        result.orientation.x = ori_x / step;
        result.orientation.y = ori_y / step;
        result.orientation.z = ori_z / step;
        result.orientation.w = ori_w / step;

        output.push_back(result);
    }
}

void write_cart(ofstream& os, string classification) {
    list<Pose>::const_iterator it = cartesian_temp.begin();
    
    while (it != cartesian_temp.end()) {
        os << it->position.x << " " << it->position.y << " " << it->position.z << " "
           << it->orientation.x << " " << it->orientation.y << " " << it->orientation.z << " " << it->orientation.w << " ";
        it++;
    }

    os << classification << endl;

}

/**
 * Writes the temporal bins for each joint and the classification for the action
 * Writes j1.vel j1.pos j1.eff j2.vel... for each temporal bin
 */
void write_list(ofstream& os, string classification) {
    Dataset::data_list_cit j1 = joint_1_temp.begin();
    Dataset::data_list_cit j2 = joint_2_temp.begin();
    Dataset::data_list_cit j3 = joint_3_temp.begin();
    Dataset::data_list_cit j4 = joint_4_temp.begin();
    Dataset::data_list_cit j5 = joint_5_temp.begin();
    Dataset::data_list_cit j6 = joint_6_temp.begin();
    Dataset::data_list_cit f1 = finger_1_temp.begin();
    Dataset::data_list_cit f2 = finger_2_temp.begin();

    // Printing the temporal bins
    for (int i = 0; i < 10; i++) {
        os << j1->vel << " " << j1->pos << " " << j1->eff << " ";
        os << j2->vel << " " << j2->pos << " " << j2->eff << " ";
        os << j3->vel << " " << j3->pos << " " << j3->eff << " ";
        os << j4->vel << " " << j4->pos << " " << j4->eff << " ";
        os << j5->vel << " " << j5->pos << " " << j5->eff << " ";
        os << j6->vel << " " << j6->pos << " " << j6->eff << " ";
        os << f1->vel << " " << f1->pos << " " << f1->eff << " ";
        os << f2->vel << " " << f2->pos << " " << f2->eff << " ";
        j1++;
        j2++;
        j3++;
        j4++;
        j5++;
        j6++;
        f1++;
        f2++;
    }

    // Printing the classification
    os << classification << endl;
}

/**
 * Checks if the passed filename is the name of a file that exists
 * Returns true if the file exists, false otherwise
 */
bool file_exists(const string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}

/**
 * Clears the lists for the joints and the temporal bins for each joint
 */
void clear_lists() {
    joint_1.clear();
    joint_1_temp.clear();
    joint_2.clear();
    joint_2_temp.clear();
    joint_3.clear();
    joint_3_temp.clear();
    joint_4.clear();
    joint_4_temp.clear();
    joint_5.clear();
    joint_5_temp.clear();
    joint_6.clear();
    joint_6_temp.clear();
    finger_1.clear();
    finger_1_temp.clear();
    finger_2.clear();
    finger_2_temp.clear();
    cartesian.clear();
    cartesian_temp.clear();
}

/**
 * Calls split for each of the joints to create the temporal bins
 */
void split_lists() {
    split(joint_1, joint_1_temp);
    split(joint_2, joint_2_temp);
    split(joint_3, joint_3_temp);
    split(joint_4, joint_4_temp);
    split(joint_5, joint_5_temp);
    split(joint_6, joint_6_temp);
    split(finger_1, finger_1_temp);
    split(finger_2, finger_2_temp);
    split(cartesian, cartesian_temp);
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
    if (repeat != "Y" && repeat != "y") {
        return false;
    }

    return true;
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
 * Joins the lists of temporal bins into one list
 */
Dataset::action_list join_lists() {
    Dataset::action_list result;

    Dataset::data_list_cit j1 = joint_1_temp.begin();
    Dataset::data_list_cit j2 = joint_2_temp.begin();
    Dataset::data_list_cit j3 = joint_3_temp.begin();
    Dataset::data_list_cit j4 = joint_4_temp.begin();
    Dataset::data_list_cit j5 = joint_5_temp.begin();
    Dataset::data_list_cit j6 = joint_6_temp.begin();
    Dataset::data_list_cit f1 = finger_1_temp.begin();
    Dataset::data_list_cit f2 = finger_2_temp.begin();

    // Printing the temporal bins
    for (int i = 0; i < 10; i++) {
        result.push_back(*j1);
        result.push_back(*j2);
        result.push_back(*j3);
        result.push_back(*j4);
        result.push_back(*j5);
        result.push_back(*j6);
        result.push_back(*f1);
        result.push_back(*f2);
        j1++;
        j2++;
        j3++;
        j4++;
        j5++;
        j6++;
        f1++;
        f2++;
    }

    return result;
}

/**
 * Displays error when incorrect command line args given
 */
void print_err() {
    ROS_ERROR("Usage: rosrun lfd_actions knn -src <dataset file>"
              "\n\t\t\t\t(Optional)\n\t\t\t\t -super <true/false>");
}

int main(int argc, char** argv) {
    // Initializing the ros node
    ros::init(argc, argv, "arff_recorder");
    ros::NodeHandle n;

    // Getting command line arguments
    string dataset_name;
    bool supervised = false;
    bool found_d = false;
    for (int i = 1; i < argc; i++) {
        string argv_str(argv[i]);
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
    ifstream data_file(dataset_name.c_str());
    Dataset data(data_file, 3);
    data.print_dataset();

    // Creating the subscriber
    ros::Subscriber arm_sub = n.subscribe(ARM_TOPIC, 1000, arm_cb);
    ros::Subscriber cart_sub = n.subscribe(CART_TOPIC, 1000, cart_cb);

    // Opening ofstream for the dataset
    ofstream os;
    if (supervised) {
        if (file_exists(dataset_name)) {
            os.open(dataset_name.c_str(), std::ios_base::app);
        } else {
            os.open(dataset_name.c_str());
        }
    }

    bool again = true;
    while (again) {
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

        // Put into temporal bins
        split_lists();

        // Perform k-NN on recorded action
        //Dataset::action_list set = join_lists();
       // string guess = data.guess_classification(set);
        //string guess_alt = data.guess_classification_alt(set);
        string guess = data.guess_classification_cart(cartesian_temp);
	ROS_INFO("%s", guess.c_str());
        

        // Print out the guess for the action
        print_guess(guess);

        // If supervised checking guess with user
        if (supervised) {
            // Getting the correct label
            string label = confirm_guess(guess);

            // Printing the lists
            //write_list(os, label);
            write_cart(os, label);

            /*
            Dataset::action recorded;
            recorded.classification = label;
            recorded.data = set;
            */
            Dataset::action_cart recorded;
            recorded.classification = label;
            recorded.cartesian = cartesian_temp;
        }

        // Clearing the lists
        clear_lists();

        // Getting whether or not to record another action
        again = repeat();
    }

    os.close();

    return 0;
}
