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
using std::endl;
using std::istream;
using std::ifstream;
using std::ofstream;
using std::vector;
using std::list;
using std::map;

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
map<string, list<coord> > joints;
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

// Data set lists
list<list<coord> > lifts;
list<list<coord> > sweeps;

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

void arm_cb2(const sensor_msgs::JointState::ConstPtr& msg) {
    vector<string> names = msg->name;

    if (names.size() == 8) {
        vector<double> positions = msg->position;
        vector<double> velocities = msg->velocity;
        vector<double> efforts = msg->effort;
        
        int i = 0;
        for (vector<string>::const_iterator it = names.begin();
                it != names.end(); it++) {
            coord state;
            state.pos = positions[i];
            state.vel = velocities[i];
            state.eff = efforts[i];
    
            joints[*it].push_back(state);
            i++;
        }
    }
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
            coord state;
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

/**
 * Splits the raw_data into temporal bins into the output
 */
void split(list<coord>& raw_data, list<coord>& output) {
    list<coord>::size_type size = raw_data.size();
    int step = std::ceil(size / 10.0);

    // Splitting the list into 10 sets and looping through each set
    for (list<coord>::size_type start = 0; start < raw_data.size(); start += step) {
        double pos_sum = 0;
        double vel_sum = 0;
        double eff_sum = 0;

        // Moving it to the beginning of the set
        list<coord>::const_iterator it = raw_data.begin();
        for (int i = 0; i < start; i++) {
            it++;
        }

        // Moving stop to the end of the set
        list<coord>::const_iterator stop = it;
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
        coord result;
        result.pos = pos_sum / step;
        result.vel = vel_sum / step;
        result.eff = eff_sum / step;
        output.push_back(result);
    }
}

/*
void write_list2(ofstream& os, string classification) {
    typedef map<string, list<coord> >::const_iterator map_it;

    for (map_it it = joints.begin(); it != joints.end(); it++) {
        list<coord> vals = it->second();
    }
}
*/

/**
 * Writes the temporal bins for each joint and the classification for the action
 * Writes j1.vel j1.pos j1.eff j2.vel... for each temporal bin
 */
void write_list(ofstream& os, string classification) {
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
}

/**
 * Prompts the user to record another actions
 * Returns true if going to record another action, false otherwise
 */
bool repeat() {
    string repeat;
    cout << "Again [Y/y]: ";
    cin >> repeat;
    if (repeat != "Y" && repeat != "y") {
        return false;
    }

    return true;;
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

bool is_num(char c) {
    return isdigit(c) || c == '.';
}

string split_line(const string& line, vector<float>& values) {
    string::const_iterator begin = line.begin();
    while (is_num(*begin)) {
        string::const_iterator end = begin;
        while (is_num(*end)) {
            end++;
        }

        string val(begin, end);
        values.push_back(atof(val.c_str()));
        begin = end + 1;
    }

    string classification(begin, line.end());
    return classification;
}

list<coord> build_coord_list(const vector<float>& values) {
    list<coord> coords;

    for (int i = 0; i < values.size(); i += 3) {
        coord c;
        c.vel = values[i];
        c.pos = values[i + 1];
        c.eff = values[i + 2];
        coords.push_back(c);
    }

    return coords;
}

void build_dataset(ifstream& is) {
    while (is) {
        string line;
        getline(is, line);

        vector<float> values;
        string classification = split_line(line, values);
        list<coord> coords = build_coord_list(values);

        if (classification == "lift") {
            lifts.push_back(coords);
        } else if (classification == "sweep") {
            sweeps.push_back(coords);
        }
    }
}

void print_list(const string& classification, const list<coord>& l) {
    for (list_it it = l.begin(); it != l.end(); it++) {
        cout << it->vel << " " << it->pos << " " << it->eff << " ";
    }

    cout << classification << endl;
}

void print_dataset() {
    for (list<list<coord> >::const_iterator it = lifts.begin(); it != lifts.end(); it++) {
        print_list("lift", *it);
    }

    for (list<list<coord> >::const_iterator it = sweeps.begin(); it != sweeps.end(); it++) {
        print_list("sweep", *it);
    }
}

int main(int argc, char** argv) {
    // Initializing the ros node
    ros::init(argc, argv, "arff_recorder");
    ros::NodeHandle n;

    // Getting the input dataset file
    string dataset_name = argv[1];
    ifstream dataset(dataset_name.c_str());
    build_dataset(dataset);

    // Getting whether or not to supervise
    bool supervised = false;
    if (argc >= 3) {
        string super = argv[2];
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

    bool again = false;
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
            write_list(os, label);
        }

        // Clearing the lists
        clear_lists();

        // Getting whether or not to record another action
        again = repeat();
    }

    os.close();

    return 0;
}
