#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <list>
#include <termios.h>
#include "sensor_msgs/JointState.h"

using std::string;
using std::cin;
using std::cout;
using std::endl;
using std::ofstream;
using std::vector;
using std::list;

// struct for the different values obtained from the joints
struct coord {
    double vel;
    double pos;
    double eff;
};

typedef list<coord>::const_iterator list_it;

// The topic the arm joints publish to
const string ARM_TOPIC = "/joint_states";

// lists of joint_states
list<coord> joint_1;
list<coord> joint_2;
list<coord> joint_3;
list<coord> joint_4;
list<coord> joint_5;
list<coord> joint_6;
list<coord> finger_1;
list<coord> finger_2;

// lists of temporal bins for the coord
list<coord> joint_1_temp;
list<coord> joint_2_temp;
list<coord> joint_3_temp;
list<coord> joint_4_temp;
list<coord> joint_5_temp;
list<coord> joint_6_temp;
list<coord> finger_1_temp;
list<coord> finger_2_temp;

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
            coord state;
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
void split(list<coord>& raw_data, list<coord>& output) {
    list<coord>::size_type size = raw_data.size();
    int step = std::ceil(size / 10.0);

    for (list<coord>::size_type start = 0; start < raw_data.size(); start += step) {
        double pos_sum = 0;
        double vel_sum = 0;
        double eff_sum = 0;

        list<coord>::const_iterator it = raw_data.begin();
        for (int i = 0; i < start; i++) {
            it++;
        }

        list<coord>::const_iterator stop = it;
        for (int i = 0; i < step; i++) {
            stop++;
        }

        while (it != stop && it != raw_data.end()) {
            pos_sum += it->pos;
            vel_sum += it->vel;
            eff_sum += it->eff;
            it++;
        }

        coord result;
        result.pos = pos_sum / step;
        result.vel = vel_sum / step;
        result.eff = eff_sum / step;

        output.push_back(result);
    }
}

void print_list(ofstream& os, string classification) {
    list_it j1 = joint_1_temp.begin();
    list_it j2 = joint_2_temp.begin();
    list_it j3 = joint_3_temp.begin();
    list_it j4 = joint_4_temp.begin();
    list_it j5 = joint_5_temp.begin();
    list_it j6 = joint_6_temp.begin();
    list_it f1 = finger_1_temp.begin();
    list_it f2 = finger_2_temp.begin();

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

bool file_exists(const string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}

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
}

void split_lists() {
    split(joint_1, joint_1_temp);
    split(joint_2, joint_2_temp);
    split(joint_3, joint_3_temp);
    split(joint_4, joint_4_temp);
    split(joint_5, joint_5_temp);
    split(joint_6, joint_6_temp);
    split(finger_1, finger_1_temp);
    split(finger_2, finger_2_temp);
}

bool repeat() {
    string repeat;
    cout << "Again [Y/y]: ";
    cin >> repeat;
    if (repeat != "Y" && repeat != "y") {
        return false;
    }

    return true;;
}

void print_guess(const string& guess) {
    cout << "Action guess: " << guess << endl;
}

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

int main(int argc, char** argv) {
    // Initializing the ros node
    ros::init(argc, argv, "arff_recorder");
    ros::NodeHandle n;

    // Getting whether or not to supervise
    bool supervised = false;
    if (argc >= 2) {
        string super = argv[1];
        supervised = super == "supervise";
        ROS_INFO("supervise");
    }

    // Creating the subscriber
    ros::Subscriber arm_sub = n.subscribe(ARM_TOPIC, 1000, arm_cb);

    // Setting the filenames
    string file = "dataset/actions.txt";

    // Opening ofstream for the dataset
    ofstream os;
    if (file_exists(file)) {
        os.open(file.c_str(), std::ios_base::app);
    } else {
        os.open(file.c_str());
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
        string guess;
        // TODO perform k-NN
        
        // Print out the guess for the action
        print_guess(guess);

        // If supervised checking guess with user
        if (supervised) {
            // Getting the correct label
            string label = confirm_guess(guess);

            // Printing the lists
            print_list(os, label);
        }

        // Clearing the lists
        clear_lists();

        // Getting whether or not to record another action
        again = repeat();
    }

    os.close();

    return 0;
}