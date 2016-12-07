#include "dataset.h"
#include <cmath>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <iostream>

using std::ifstream;
using std::ofstream;
using std::cout;
using std::endl;
using std::list;
using std::vector;
using std::string;
using std::map;
using geometry_msgs::Point;

Dataset::Dataset(const string& file, bool verbose, int k = 1) : base_k(k) {
    map<string, int> counts;

    // Setting up streams
    ifstream is(file.c_str());
    os.open(file.c_str(), std::ios_base::app);

    if (is) {
        // Getting the line
        string line;
        getline(is, line);

        if (line != "") {
            // Getting the base Action to offset the others
            Action ac(line);
            base = ac.pose_begin()->position;
            action_set.push_back(ac);

            if (verbose) {
                counts[ac.get_label()]++;
            }

            // Looping while not at end of file
            while (is) {
                // Getting the line
                string line;
                getline(is, line);

                // If line not blank build the action and push it
                if (line != "") {
                    Action ac(line);
                    ac.offset(base);
                    action_set.push_back(ac);

                    if (verbose) {
                        counts[ac.get_label()]++;
                    }
                }
            }
        }
    }

    if (verbose) {
        for (map<string, int>::const_iterator it = counts.begin();
                it != counts.end(); it++) {
            ROS_INFO("%s: %d", it->first.c_str(), it->second);
        }

        std::cout << std::endl;
    }
}

Dataset::~Dataset() {
    os.close();
}

Point Dataset::get_offset() const {
    return base;
}

void Dataset::update(Action& ac) {
    // Printing to file
    ac.print(os);

    // Adding to working dataset
    ac.offset(base);
    action_set.push_back(ac);
}

string Dataset::guess_classification(Action ac, bool verbose) {
    ac.offset(base);
    return guess_classification(ac, base_k, verbose);
}

string Dataset::guess_classification(const Action& ac, int k, bool verbose) {
    if (k == 0) {
        // Base case: empty dataset
        return "";
    }

    vector<double> closest_dist;
    vector<string> closest_str;
    vector<int> closest_num;
    int count= 1;

    if (verbose) {
        ROS_INFO("Recorded action size: %d", ac.size());
    }

    // Looping through each action in the dataset
    for (Action::action_cit it = action_set.begin();
            it != action_set.end(); it++) {
        // Getting the distance between ac and current action
        double dist = ac.get_dist(*it);

        // If verbose printing output
        if (verbose) {
            ROS_INFO("Action %d (%s): %d items: %f", count,
                    it->get_label().c_str(), it->size(), dist);
        }

        // Checking to place distance
        string label = it->get_label();
        bool placed = insert_dist(dist, label, closest_dist, closest_str, closest_num, count);

        // Checking conditions of vectors
        if (!placed && closest_dist.size() < k) {
            // Have less than k elements, adding to end
            closest_dist.push_back(dist);
            closest_str.push_back(label);
            closest_num.push_back(count);
        } else if (closest_dist.size() > k) {
            // Have more than k elements, deleting last one
            closest_dist.pop_back();
            closest_str.pop_back();
            closest_num.pop_back();
        }

        count++;
    }

    // Printing closest actions
    if (verbose) {
        for (int i = 0; i < closest_num.size(); i++) {
            ROS_INFO("%d.) Action %d: %f (%s)", i + 1, closest_num[i],
                    closest_dist[i], closest_str[i].c_str());
        }

        cout << endl;
    }

    // Getting the guess
    string guess = get_max_in_map(get_counts(closest_str));

    if (guess == "") {
        // Had a tie, reducing k by 1 until tie is broken
        if (verbose) {
            ROS_INFO("Breaking tie. Setting k to %d", k - 1);
        }

        return guess_classification(ac, k - 1, verbose);
    } else {
        return guess;
    }
}

bool Dataset::insert_dist(double dist, const string& label,
        vector<double>& closest_dist, vector<string>& closest_str,
        vector<int>& closest_num, int count) {
    bool found = false;
    vector<double>::iterator dist_it = closest_dist.begin();
    vector<string>::iterator str_it = closest_str.begin();
    vector<int>::iterator ac_it = closest_num.begin();

    // Looping through closest actions to see if need to place
    while (!found && dist_it != closest_dist.end()) {
        // If smaller distance insert it
        if (dist < *dist_it) {
            closest_dist.insert(dist_it, dist);
            closest_str.insert(str_it, label);
            closest_num.insert(ac_it, count);
            found = true;
        }

        dist_it++;
        str_it++;
        ac_it++;
    }

    return found;
}

map<string, int> Dataset::get_counts(const vector<string>& values) {
    map<string, int> counts;
    for (vector<string>::const_iterator it = values.begin();
            it != values.end(); it++) {
        counts[*it]++;
    }

    return counts;
}

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
