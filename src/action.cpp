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

typedef list<Pose>::const_iterator pose_list_cit;

Action::Action(const string& line) {
    // Getting the label and values
    vector<double> values;
    label = split_line(line, values);

    // Looping through all the values
    for (int index = 0; index < values.size(); index += 32) {
        times.push_back(ros::Time(values[index]));
        joints.push_back(get_joint_state(values, index + 1));
        poses.push_back(get_pose(values, index + 25));
    }
}

Action::Action(const std::list<geometry_msgs::Pose>& pose_list,
               const std::list<sensor_msgs::JointState> joint_list, 
               const std::list<ros::Time>& time_list) :
        poses(pose_list.begin(), pose_list.end()),
        joints(joint_list.begin(), joint_list.end()),
        times(time_list.begin(), time_list.end()) {}


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

void Action::print_jointstate(std::ofstream& os,
        const sensor_msgs::JointState& joint) const {
	for(int i = 0; i < NUM_JOINTS; i++) {	
		os << joint.position[i] << ",";
		os << joint.velocity[i]<< ",";
		os << joint.effort[i]<< "," ;
	}
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
