#include "action.h"
#include "dtw.h"
#include <cmath>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>

#define NUM_JOINTS 8
#define NUM_BINS 10

using std::ofstream;
using std::endl;
using std::string;
using std::vector;
using std::list;
using geometry_msgs::Pose;
using geometry_msgs::Point;
using sensor_msgs::JointState;

Action::Action(const string& line) {
    // Getting the label and values
    vector<double> values;
    label = split_line(line, values);

    // Looping through all the values
    for (int index = 1; index < values.size(); index += 32) {
        joints.push_back(get_joint_state(values, index));
        poses.push_back(get_pose(values, index + 24));
    }
}

Action::Action(const std::list<geometry_msgs::Pose>& input) {
    // Iterate through input list and push back
    for(list<Pose>::const_iterator input_it = input.begin();
            input_it != input.end(); input_it++) {
        poses.push_back(*input_it);
    }
}


Action::Action(const std::list<geometry_msgs::Pose>& a, const std::list<sensor_msgs::JointState> b, const std::list<ros::Time>& c) {


    for(list<Pose>::const_iterator a_it = a.begin(); a_it != a.end(); a_it++) {
        poses.push_back(*a_it);
    }


    for(list<JointState>::const_iterator b_it = b.begin(); b_it != b.end(); b_it++) {
        joints.push_back(*b_it);
    }

    for(list<ros::Time>::const_iterator c_it = c.begin(); c_it != c.end(); c_it++) {
        times.push_back(*c_it);
    }

}


void Action::print_jointstate(std::ofstream& os, const sensor_msgs::JointState& b) const {

	int i;

	for(i = 0; i < NUM_JOINTS; i++) {	

		os << b.position[i] << ",";

		os << b.velocity[i]<< ",";

		os << b.effort[i]<< "," ;
	}
}

string Action::get_label() const {
    return label;
}

int Action::size() const {
    return poses.size();
}

Action::pose_cit Action::pose_begin() const {
    return poses.begin();
}

Action::pose_cit Action::pose_end() const {
    return poses.end();
}

vector<JointState>::const_iterator Action::joint_begin() const {
    return joints.begin();
}

vector<JointState>::const_iterator Action::joint_end() const {
    return joints.end();
}

double Action::get_dist(const Action& currentAction) const {
    return DTW::min_diff(*this, currentAction);
}

void Action::print(ofstream& os) const {
    // Adding to dataset file

    int i;
    for (i = 0; i < poses.size(); i++) {
	
	os << times[i] << ",";
	print_jointstate(os, joints[i]);
        // Writing the pose
        print_pose(os, poses[i]);
	
    }
    // Writing the label
    os << label << endl;
}

void Action::set_label(const string& new_label) {
    label = new_label;
}

void Action::offset(const Point& base) {
    typedef vector<Pose>::iterator pose_it;
    for (pose_it it = poses.begin(); it != poses.end(); it++) {
        it->position.x -= base.x;
        it->position.y -= base.y;
        it->position.z -= base.z;
    }
}


void Action::print_pose(ofstream& os, const Pose& pose) const {
    os << pose.position.x << "," << pose.position.y << "," << pose.position.z
       << "," << pose.orientation.x << "," << pose.orientation.y << ","
       << pose.orientation.z << "," << pose.orientation.w << ",";
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

JointState Action::get_joint_state(const std::vector<double>& values, int i) {
    JointState js;

    for (int j = 0; j < 24; j += 3) {
        js.position.push_back(values[i + j]);
        js.velocity.push_back(values[i + j+ 1]);
        js.effort.push_back(values[i + j + 2]);
    }

    return js;
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

/**
 * Calculates the distance between two joint_states
 * Returns the distance
 */
/*
double Action::joint_dist(const joint_state& data, const joint_state& recorded)  const {
    double dvel = pow(recorded.vel - data.vel, 2);
    double dpos = pow(recorded.pos - data.pos, 2);
    double deff = pow(recorded.eff - data.eff, 2);

    return sqrt(dvel + dpos + deff);
}
*/

/**
 * Calculates the euclidean distance between two Points
 * Returns the distance
 */
double Action::euclidean_dist(const geometry_msgs::Point& data,
        const geometry_msgs::Point& action) const {
    double dx = pow(action.x - data.x, 2);
    double dy = pow(action.y - data.y, 2);
    double dz = pow(action.z - data.z, 2);
    double dist = sqrt(dx + dy + dz);

    // Returning the sum of the distances
    return dist;
}

/**
 * Calculates the distance between two Quaternions
 * Returns the distance
 */
double Action::quarterion_dist(const geometry_msgs::Quaternion& c,
        const geometry_msgs::Quaternion& d) const {
    Eigen::Vector4f dv;
    dv[0] = d.w; dv[1] = d.x; dv[2] = d.y; dv[3] = d.z;
    Eigen::Matrix<float, 3,4> inv;
    inv(0,0) = -c.x; inv(0,1) = c.w; inv(0,2) = -c.z; inv(0,3) = c.y;
    inv(1,0) = -c.y; inv(1,1) = c.z; inv(1,2) = -c.w; inv(1,3) = -c.x;
    inv(2,0) = -c.z; inv(2,1) = -c.y; inv(2,2) = -c.x; inv(2,3) = c.w;

    Eigen::Vector3f m = inv * dv * -2.0;
    return m.norm();
}
