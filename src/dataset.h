#ifndef GUARD_dataset_h
#define GUARD_dataset_h

#include <iostream>
#include <fstream>
#include <list>
#include <map>
#include <string>
#include <vector>

/**
 * Class to represent the dataset that the robot uses to classify actions
 * Stores the various example actions and provides a method to get the
 * closest match using a k-NN algorithm to get the closest matching action
 */
class Dataset {
    public:
        // Struct for the data points for each temporal bin
        struct data_point {
            double vel;
            double pos;
            double eff;
        };

        // Struct for the data points with a label for the classification
        struct labeled_data_point {
            std::string label;
            data_point data;
        };

        // List type being used for a Dataset
        typedef std::list<data_point> action_list;
        typedef std::list<labeled_data_point> labeled_action_list;

        // Struct for the list of data for an action
        struct action {
            std::string classification;
            action_list data;
        };

        // List of actions
        typedef std::list<action> action_set;

        // Defining iterators for the internal values
        typedef action_list::const_iterator data_list_cit;
        typedef labeled_action_list::const_iterator l_data_list_cit;
        typedef action_set::const_iterator data_group_cit;

        /**
         * Builds a Dataset given an input stream to a file
         * The file should have the data for each action on a line with each
         * data point separated by a single whitespace. The line should end
         * with the classification for the action
         */
        Dataset(std::ifstream&);

        /**
         * Prints out each actions data_points and classification to the
         * standard output stream
         */
        void print_dataset();

        /**
         * Guesses the classification for the given action using k-NN to
         * get the nearest neighbor for each temporal bin
         * Returns the guessed classifaction
         */
        std::string guess_classification(const action_list&);

    private:
        // The lists of each action based on classification
        action_set actions;

        /**
         * Gets whether or not character is part of a number
         * Returns true if character is a digit or '.', false otherwise
         */
        bool is_num(char);

        /**
         * Builds a data_point list from a vector with all the values
         * The values are stored velocity, position, effort for each bin
         * with all bins for time step 1 then followed by time step 2, etc
         * Returns the data_point list that was built
         */
        action_list build_bin_list(const std::vector<float>&);

        /**
         * Splits a string up into a vector of floats
         * Splits the string using single whitespace characters
         * The line should end with the classification for the data
         * Returns the split string through the vector reference and returns
         * the classication for the data
         */
        std::string split_line(const std::string&, std::vector<float>&);

        /**
         * Gets the distance between two data_points by extending the
         * Pythagorean theorum to three points
         * Returns the distance between the two points
         */
        double get_dist(const data_point&, const data_point&);

        /**
         * Gets the classification for the passed data_point based on the other
         * data_points from actions in the dataset from the same temporal bin
         * Returns the name of the classification based on the one temporal bin
         */
        std::string bin_classification(const data_point&, const labeled_action_list&);

        /**
         * Gets the data_points from an action_set from the specified joint and
         * temporal bin
         * Returns a labeled_action_list of data_points for a single joint
         * and time
         */
        labeled_action_list get_bin(int, int, const action_set&);

        /**
         * Gets the key to the maximum value in a map of strings to ints
         * Returns the key with the max value
         */
        std::string get_max_in_map(const std::map<std::string, int>&);

        /**
         * Prints out one of the action's data points and it's classification
         */
        void print_list(const action&);

};

#endif
