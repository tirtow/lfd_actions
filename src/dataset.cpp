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

Dataset::Dataset(const string& file, int k = 1) : base_k(k) {
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
    return guess_classification(ac, base_k);
}

string Dataset::guess_classification(const Action& ac, int k) {
    vector<double> closest_dist;
    vector<string> closest_str;

    // Looping through each action in the dataset
    for (Action::action_cit it = action_set.begin();
            it != action_set.end(); it++) {
        // Getting the distance between ac and current action
        double dist = ac.get_dist(*it);

        // Checking to place distance
        string label = it->get_label();
        bool placed = insert_dist(dist, label, closest_dist, closest_str);

        // Checking conditions of vectors
        if (!placed && closest_dist.size() < k) {
            // Have less than k elements, adding to end
            closest_dist.push_back(dist);
            closest_str.push_back(label);
        } else if (closest_dist.size() > k) {
            // Have more than k elements, deleting last one
            closest_dist.pop_back();
            closest_str.pop_back();
        }
    }

    // Getting the guess
    string guess = get_max_in_map(get_counts(closest_str));

    if (guess == "") {
        // Had a tie, reducing k by 1 until tie is broken
        ROS_INFO("Breaking tie. Setting k to %d", k - 1);
        return guess_classification(ac, k - 1);
    } else {
        return guess;
    }
}

bool Dataset::insert_dist(double dist, const string& label,
        vector<double>& closest_dist, vector<string>& closest_str) {
    bool found = false;
    vector<double>::iterator dist_it = closest_dist.begin();
    vector<string>::iterator str_it = closest_str.begin();

    // Looping through closest actions to see if need to place
    while (!found && dist_it != closest_dist.end()) {
        // If smaller distance insert it
        if (dist < *dist_it) {
            closest_dist.insert(dist_it, dist);
            closest_str.insert(str_it, label);
            found = true;
        }

        dist_it++;
        str_it++;
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
