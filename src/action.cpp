#include <cmath>

#define NUM_JOINTS 8
#define NUM_BINS 10

using std::ofstream;
using std::endl;
using std::string;
using std::vector;
using geometry_msgs::Pose;

Action::Action(const string& line) {
    // Getting the label and values
    vector<double> values;
    label = split_line(line, values);

    // Looping through all the values
    int index = 0;
    while (index < values.size()) {
        // Creating the bin
        bin b;

        // Building the joint_states
        for (int i = index; i < 24; i += 3) {
            b.joints.push_back(get_joint_state(values, i));
        }

        // Getting the pose
        b.pose = get_pose(values, index++);

        // Adding to the action
        data.push_back(b);
    }
}

Action::Action(const vector<Pose>& poses, const joint_list& joints) {
    // Blank label for unlabeled action
    label = "";

    // Building the bins
    for (int bin = 0; bin < NUM_BINS; bin++) {
        data.push_back(build_bin(bin, poses, joints));
    }
}

void Action::set_label(const string& new_label) {
    label = new_label;
}

void Action::print(ofstream& os) const {
    // Adding to dataset file
    for (bin_cit bin_it = data.begin(); bin_it != data.end(); bin_it++) {
        // Writing the joint_states
        for (joint_cit joint_it = bin_it->joints.begin();
                joint_it != bin_it->joints.end(); joint_it++) {
            os << joint_it->vel << " " << joint_it->pos << " "
               << joint_it->eff << " ";
        }

        // Writing the pose
        print_pose(os, bin_it->pose);
    }

    // Writing the label
    os << label << endl;
}

void Action::print_pose(ofstream& os, const Pose& pose) {
    os << pose.position.x << " " << pose.position.y << " " << pose.position.z
       << " " << pose.orientation.x << " " << pose.orientation.y << " "
       << pose.orientation.z << " " << pose.orientation.w;
}

bool Action::is_num(char c) {
    return isdigit(c) || c == '.' || c == 'e' || c== '+' || c == '-';
}

string Action::split_line(const string& line, vector<double>& values) {
    // Iterating over entire line
    string::const_iterator begin = line.begin();
    while (is_num(*begin)) {
        // Moving end down until at next non-digit character
        string::const_iterator end = begin;
        while (is_num(*end)) {
            end++;
        }

        // Building string from iterators and pushing double value to vector
        string val(begin, end);
        values.push_back(atof(val.c_str()));

        // Moving begin to one past end
        begin = end + 1;
    }

    // Getting the classification from the remaining characters
    string classification(begin, line.end());

    return classification;
}

Action::bin Action::build_bin(int bin_num, const vector<Pose>& poses, const joint_list& joints) {
    bin b;

    // Getting the pose
    b.pose = poses[bin_num];

    // Getting the joints
    int offset = bin_num * NUM_BINS;
    for (int i = 0; i < NUM_JOINTS; i++) {
        b.joints.push_back(joints[i + offset]);
    }

    return b;
}

Action::joint_state Action::get_joint_state(const vector<double>& values, int i) {
    joint_state state;
    state.vel = values[i];
    state.pos = values[i + 1];
    state.eff = values[i + 2];

    return state;
}

Pose Action::get_pose(const std::vector<double>& values, int i) {
    Pose pose;

    // Getting the position
    pose.position.x = values[i];
    pose.position.y = values[i + 1];
    pose.position.z = values[i + 2];

    // Getting the orientation
    pose.orientation.x = values[i + 3];
    pose.orientation.y = values[i + 4];
    pose.orientation.z = values[i + 5];
    pose.orientation.w = values[i + 6];

    return pose;
}
