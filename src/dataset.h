#ifndef GUARD_dataset_h
#define GUARD_dataset_h

#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <vector>
#include "action.h"

/**
 * Class to represent the dataset that the robot uses to classify actions
 * Stores the various example actions and provides a method to get the
 * closest match using a k-NN algorithm to get the closest matching action
 */
class Dataset {
    public:
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
         * Calls the overloaded guess_classification passing base_k
         * as the value for k
         */
        std::string guess_classification(Action&, bool);

        /**
         * Gets the offset used by this Dataset
         * Returns base
         */
        geometry_msgs::Point get_offset() const;

        /**
         * Updates the dataset by adding the action to the working dataset
         * and adding it to the dataset file
         */
        void update(Action&);

    private:
        int base_k;
        std::ofstream os;
        Action::action_list action_set;
		geometry_msgs::Point base;

        /**
         * Guesses the classification of an action given the number of nearest
         * neighbors to consider. Calculates the distance between the passed
         * Action and each Action in this Dataset to find the nearest neighbors
         * Returns the guess for the classification
         */
        std::string guess_classification(const Action&, int, bool);

        /**
         * Inserts the distance and label into the respective vectors based on
         * the distance. Inserts such that the vectors are ordered from the
         * smallest distance to the greatest distance
         * Returns whether or not the value was inserted
         */
        bool insert_dist(double, const std::string&, std::vector<double>&,
                std::vector<std::string>&);

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
