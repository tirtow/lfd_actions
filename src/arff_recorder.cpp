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

typedef list<joint_state>::const_iterator list_it;

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
void split(list<joint_state>& raw_data, list<joint_state>& output) {
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
        os << j1->pos << ", " << j1->vel << ", " << j1->eff << ", ";
        os << j2->pos << ", " << j2->vel << ", " << j2->eff << ", ";
        os << j3->pos << ", " << j3->vel << ", " << j3->eff << ", ";
        os << j4->pos << ", " << j4->vel << ", " << j4->eff << ", ";
        os << j5->pos << ", " << j5->vel << ", " << j5->eff << ", ";
        os << j6->pos << ", " << j6->vel << ", " << j6->eff << ", ";
        os << f1->pos << ", " << f1->vel << ", " << f1->eff << ", ";
        os << f2->pos << ", " << f2->vel << ", " << f2->eff << ", ";
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
    os << "\'" << classification << "\'" << endl;
}

void print_attr(ofstream& os, int bin_num, int joint_num, 
        string joint, string label) {
    os << "@attribute \'bin" << bin_num << "-" << joint << joint_num
       << "-" << label << "\' numeric" << endl;
}

void print_header(ofstream& os) {
    os << "@relation actions" << endl;

    for (int bin = 1; bin <= 10; bin++) {
        for (int joint = 1; joint <= 6; joint++) {
            print_attr(os, bin, joint, "joint", "pos");
            print_attr(os, bin, joint, "joint", "vel");
            print_attr(os, bin, joint, "joint", "eff");
        }

        for (int finger = 1; finger <= 2; finger++) {
            print_attr(os, bin, finger, "finger", "pos");
            print_attr(os, bin, finger, "finger", "vel");
            print_attr(os, bin, finger, "finger", "eff");
        }
    }

    os << "@attribute \'Class\' { \'lift\', \'sweep\' }" << endl
       << "@data" << endl;
}

bool file_exists(const string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}

int main(int argc, char** argv) {
    // Initializing the ros node
    ros::init(argc, argv, "arff_recorder");
    ros::NodeHandle n;

    // Creating the subscriber
    ros::Subscriber arm_sub = n.subscribe(ARM_TOPIC, 1000, arm_cb);

    // Setting the filenames
    string file = "actions.arff";

    // Waiting to record
    string classification;
    cout << "Enter classification: ";
    cin >> classification;
    
    // Recording the data
    while (ros::ok() && getch() != 'q') {
        ros::spinOnce();
    }

    // Put into temporal bins
    split(joint_1, joint_1_temp);
    split(joint_2, joint_2_temp);
    split(joint_3, joint_3_temp);
    split(joint_4, joint_4_temp);
    split(joint_5, joint_5_temp);
    split(joint_6, joint_6_temp);
    split(finger_1, finger_1_temp);
    split(finger_2, finger_2_temp);

    // Dumping data
    ofstream os;

    if (file_exists(file)) {
        os.open(file.c_str(), std::ios_base::app);
    } else {
        os.open(file.c_str());
        
        // Printing the arff header
        print_header(os);
    }

    // Printing the lists
    print_list(os, classification);

    os.close();

    return 0;
}
