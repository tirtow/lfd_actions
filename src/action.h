#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <fstream>
#include <list>
#include <string>
#include <vector>
#include <std_msgs/Time.h>

#ifndef GUARD_action_h
#define GUARD_action_h

/**
 * Class to represent a recorded Action from the arm
 * Stores the times, joint states, poses, and the label for the
 * Action
 */
class Action {
    public:
        // List types
        typedef std::vector<geometry_msgs::Pose> pose_list;
        typedef std::vector<sensor_msgs::JointState> joint_list;
        typedef std::vector<ros::Time> time_list;
        typedef std::vector<Action> action_list;

        // List iterator types
        typedef pose_list::const_iterator pose_cit;
        typedef joint_list::const_iterator joint_cit;
        typedef time_list::const_iterator time_cit;
        typedef action_list::const_iterator action_cit;

        /**
         * Builds an action given a string with the values for the action
         * and the label for the action
         */
        Action(const std::string&);

        /**
         * Builds an Action given a list of cartesian poses, joint states,
         * and times
         */
        Action(const std::list<geometry_msgs::Pose>&,
               const std::list<sensor_msgs::JointState>,
               const std::list<ros::Time>&);

        /**
         * Gets the label for this Action
         * Returns label
         */
        std::string get_label() const;

        /**
         * Gets the number of poses in this Action
         * Returns poses.size()
         */
        int size() const;

        /**
         * Gets an iterator to the beginning of poses
         * Returns poses.begin()
         */
        pose_cit pose_begin() const;

        /**
         * Gets an iterator to the end of poses
         * Returns poses.end()
         */
        pose_cit pose_end() const;

        /**
         * Gets an iterator to the begining of joints
         * Returns joints.begin()
         */
        joint_cit joint_begin() const;

        /**
         * Gets an iterator to the end of joints
         * Returns joints.end()
         */
        joint_cit joint_end() const;

        /**
         * Calculates the distance between this Action and another Action
         */
        double get_dist(const Action&) const;

        /**
         * Sets the label for this Action
         */
        void set_label(const std::string&);

        /**
         * Prints this Action to the specified output stream
         */
        void print(std::ofstream&) const;

        /**
         * Offsets the cartesian position of this Action by a Point
         */
        void offset(const geometry_msgs::Point&);

    private:
        std::string label;
        time_list times;
        pose_list poses;
        joint_list joints;

        /**
         * Prints the pose of this Action to the specified output stream
         */
        void print_pose(std::ofstream&, const geometry_msgs::Pose&) const;

        /**
         * Prints the passed joint state to the specified output stream
         */
        void print_jointstate(std::ofstream&,
                const sensor_msgs::JointState&) const;

        /**
         * Gets whether or not a character is part of a number
         * Returns true if a digit, '-', '.',  or 'e', false otherwise
         */
        bool is_num(char c);

        /**
         * Splits a string into a vector of double values and returns
         * the string label at the end of the line. Returns the vector of
         * values through a reference to the vector
         */
        std::string split_line(const std::string&, std::vector<double>&);

        /**
         * Gets the Pose from a vector of values
         */
        geometry_msgs::Pose get_pose(const std::vector<double>&, int);

        /**
         * Gets the joint state from a vector of values
         */
        sensor_msgs::JointState get_joint_state(const std::vector<double>&, int);

};

#endif
