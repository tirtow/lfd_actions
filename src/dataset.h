#ifndef GUARD_dataset_h
#define GUARD_dataset_h

#include <iostream>
#include <fstream>
#include <list>
#include <map>
#include <string>
#include <vector>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

/**
 * Class to represent the dataset that the robot uses to classify actions
 * Stores the various example actions and provides a method to get the
 * closest match using a k-NN algorithm to get the closest matching action
 */
class Dataset {
    public:
        // Struct for the joint_states for a temporal bin
        struct joint_state {
            double vel;
            double pos;
            double eff;
        };

        // Struct for a bin of an action
        struct bin {
            std::vector<joint_state> joints;
            geometry_msgs::Pose pose;
        };

        // Struct for the label and data for an action
        struct action {
            std::string label;
            std::vector<bin> data;
        };

        // List types
        typedef std::vector<joint_state> joint_list;
        typedef std::vector<geometry_msgs::Pose> pose_list;
        typedef std::vector<bin> bin_list;
        typedef std::vector<action> action_list;

        // List iterator types
        typedef joint_list::const_iterator joint_cit;
        typedef pose_list::const_iterator pose_cit;
        typedef bin_list::const_iterator bin_cit;
        typedef action_list::const_iterator action_cit;

        /**
         * Builds a Dataset given an input stream to a file
         * The file should have the data for each action on a line with each
         * data point separated by a single whitespace. The line should end
         * with the classification for the action
         * Takes an int for the number of nearest neighbors (k) to consider in
         * the k-NN classification
         */
        Dataset(const std::string&, int);

        /**
         * Destructor to close the ofstream used to write
         */
        ~Dataset();

        /**
         * Guess the classification for the given action using k-NN to
         * get the nearest neighbor using the pose of the end-effector
         * Sums the differences between each temporal bin to calculate the
         * k nearest neighbors
         * Returns the guessed classification
         */
        std::string guess_classification_cart(const pose_list&);

        /**
         * Updates the dataset by adding the action to the working dataset
         * and adding it to the dataset file
         */
        void update(const action&);

    private:
        // Iterator for a vector of doubles
        typedef std::vector<double>::const_iterator vec_it;

        // The number of neighbors to consider for k-NN
        int k;

        // The output file stream for the dataset
        std::ofstream os;

        // The list of actions that makes up the dataset
        action_list action_set;

        /**
         * Adds an action to the working dataset
         */
        void add(const action&);

        /**
         * Prints out the pose position and orientation
         */
        void print_pose(std::ofstream&, const geometry_msgs::Pose&);

        /**
         * Gets whether or not character is part of a number
         * Returns true if character is a digit or '.', false otherwise
         */
        bool is_num(char);

        /**
         * Builds the list of poses given a vector of values
         * The values are stored as position x y z orientation x y z w
         * for each temporal bin followed by the classification for that action
         * Returns the built pose list
         */
        pose_list build_bin_list_cart(const std::vector<double>&);

        /**
         * Builds the action given the line from the dataset
         * The values are stored velocity, position, effort for each bin
         * with all bins for time step 1 followed by the cartesian position x
         * y z and orientation x y z w followed by the time step 2, etc
         * Returns the built action
         */
        action build_action(const std::string&);

        /**
         * Gets the joint_state from the vector
         * Returns the joint_state
         */
        joint_state get_joint_state(const std::vector<double>&, int);

        /**
         * Gets the pose from the vector
         * Returns the pose
         */
        geometry_msgs::Pose get_pose(const std::vector<double>&, int);

        /**
         * Splits a string up into a vector of doubles
         * Splits the string using single whitespace characters
         * The line should end with the classification for the data
         * Returns the split string through the vector reference and returns
         * the classication for the data
         */
        std::string split_line(const std::string&, std::vector<double>&);

        /**
         * Gets the distance between two data_points by extending the
         * Pythagorean theorum to three points
         * Returns the distance between the two points
         */
        double get_dist(const joint_state&, const joint_state&);

        /**
         * Gets the distance between two poses by calculating the distance
         * between the positions and adding that to the distance between the
         * orientations
         * Returns the summed distance
         */
        double get_pos_dist(const geometry_msgs::Point&, const geometry_msgs::Point&);

        /**
         * Creates a map of the values in the passed vector to the number of
         * times that value occurs in the vector
         * Returns the map of string to occurances
         */
        std::map<std::string, int> get_counts(const std::vector<std::string>&);

        /**
         * Gets the key to the maximum value in a map of strings to ints
         * Returns the key with the max value
         */
        std::string get_max_in_map(const std::map<std::string, int>&);

};

#endif
