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
            action_set.push_back(Action(line));
        }
    }
}

Dataset::~Dataset() {
    os.close();
}

void Dataset::update(const Action& ac) {
    // Adding to working dataset
    action_set.push_back(ac);

    // Printing to file
    ac.print(os);
}

string Dataset::guess_classification(const Action& ac) {
    return "not implemented";
}

/*
=======
String Dataset::guess_classification_quaternion(struct bin currentBin,
         const bin_list list_of_bins) {




}
 
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
