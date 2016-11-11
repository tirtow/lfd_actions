#include "dataset.h"
#include <cmath>

using std::ifstream;
using std::ofstream;
using std::cout;
using std::endl;
using std::list;
using std::vector;
using std::string;
using std::map;
using geometry_msgs::Pose;
using geometry_msgs::Point;

// cart
Dataset::Dataset(const string& file, int k_val = 1) : k(k_val) {
    // Setting up streams
    ifstream is(file.c_str());
    os.open(file.c_str(), std::ios_base::app);

    // Looping while not at end of file
    while (is) {
        // Getting the line
        string line;
        getline(is, line);

        // If line not blank build the action and push it
        if (line != "") {
            action_set.push_back(build_action(line));
        }
    }
}

Dataset::~Dataset() {
    os.close();
}

void Dataset::update(const action& ac) {
    // Adding to working dataset
    action_set.push_back(ac);

    // Adding to dataset file
    for (bin_cit bin_it = ac.data.begin(); bin_it != ac.data.end(); bin_it++) {
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
    os << ac.label << endl;
}

void Dataset::print_pose(ofstream& os, const Pose& pose) {
    os << pose.position.x << " " << pose.position.y << " " << pose.position.z
       << " " << pose.orientation.x << " " << pose.orientation.y << " "
       << pose.orientation.z << " " << pose.orientation.w;
}

void Dataset::add(const action& recorded) {
    action_set.push_back(recorded);
}

// gen
bool Dataset::is_num(char c) {
    return isdigit(c) || c == '.' || c == 'e' || c== '+' || c == '-';
}

Dataset::action Dataset::build_action(const string& line) {
    // Getting the label and values
    vector<double> values;
    string label = split_line(line, values);

    // Building the action
    action ac;
    ac.label = label;

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
        ac.data.push_back(b);
    }

    return ac;
}

Dataset::joint_state Dataset::get_joint_state(const vector<double>& values, int i) {
    joint_state state;
    state.vel = values[i];
    state.pos = values[i + 1];
    state.eff = values[i + 2];

    return state;
}

Pose Dataset::get_pose(const vector<double>& values, int i) {
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

// gen
string Dataset::split_line(const string& line, vector<double>& values) {
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

// indiv/cart
double Dataset::get_dist(const Dataset::data_point& data,
        const Dataset::data_point& action) {
    double delta_vel = pow(action.vel - data.vel, 2);
    double delta_pos = pow(action.pos - data.pos, 2);
    double delta_eff = pow(action.eff - data.eff, 2);

    return sqrt(delta_vel + delta_pos + delta_eff);
}

// cart
double Dataset::get_pos_dist(const Point& data, const Point& action) {
    // Calculating the distance between the positions
    double dx = pow(action.position.x - data.position.x, 2);
    double dy = pow(action.position.y - data.position.y, 2);
    double dz = pow(action.position.z - data.position.z, 2);
    double dist = sqrt(dx + dy + dz);

    // Returning the sum of the distances
    return dist
}

/*
// cart
string Dataset::guess_classification_cart(const list<Pose>& recorded) {
    vector<double> closest_dist;
    vector<string> closest_str;

    // Looping through each action in the dataset
    for (cart_set_cit set_it = cartesian_actions.begin();
            set_it != cartesian_actions.end(); set_it++) {
        cartesian_action current = *set_it;
        double dist_sum = 0;

        pose_it recorded_it = recorded.begin();
        // Summing up all the distances for this action
        for (pose_it it = current.cartesian.begin();
                it != current.cartesian.end(); it++) {
            dist_sum += get_dist(*recorded_it++, *it);
        }

        // Looping through closest data_points to see if need to place
        bool found = false;
        vector<double>::iterator dist_it = closest_dist.begin();
        vector<string>::iterator str_it = closest_str.begin();
        while (!found && dist_it != closest_dist.end()) {
            // If smaller distance insert it
            if (dist_sum < *dist_it) {
                closest_dist.insert(dist_it, dist_sum);
                closest_str.insert(str_it, current.classification);
                found = true;
            }

            dist_it++;
            str_it++;
        }

        if (!found && closest_dist.size() < k) {
            // Have less than k elements, adding to end
            closest_dist.push_back(dist_sum);
            closest_str.push_back(current.classification);
        } else if (closest_dist.size() > k) {
            // Have more than k elements, deleting last one
            closest_dist.pop_back();
            closest_str.pop_back();
        }
    }

    string result = get_max_in_map(get_counts(closest_str));

    // Breaking tie if one
    if (result == "") {
    ROS_INFO("tie");
        return closest_str[0];
    } else {
        return result;
    }
}
*/

// gen
map<string, int> Dataset::get_counts(const vector<string>& values) {
    map<string, int> counts;
    for (vector<string>::const_iterator it = values.begin();
            it != values.end(); it++) {
        counts[*it]++;
    }

    return counts;
}

// gen
string Dataset::get_max_in_map(const map<string, int>& counts) {
    int max = 0;
    string max_key;

    for (map<string, int>::const_iterator it = counts.begin();
            it != counts.end(); it++) {
        if (it->second > max) {
            max = it->second;
            max_key = it->first;
        } else if (it->second == max) {
            max_key = "";
        }
    }

    return max_key;
}
