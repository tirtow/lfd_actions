#include "dataset.h"
#include <cmath>

using std::ifstream;
using std::cout;
using std::endl;
using std::list;
using std::vector;
using std::string;
using std::map;

Dataset::Dataset(ifstream& is) {
    // Looping while not at end of file
    while (is) {
        // Getting the line
        string line;
        getline(is, line);

        // Splitting the line into a vector of doubles
        vector<double> values;
        string classification = split_line(line, values);

        // Building the data_points list
        action_list data_points = build_bin_list(values);

        // Building the action
        action ac;
        ac.classification = classification;
        ac.data = data_points;

        actions.push_back(ac);
    }
}

void Dataset::print_dataset() {
    for (data_group_cit it = actions.begin(); it != actions.end(); it++) {
        print_list(*it);
    }
}

string Dataset::guess_classification(const action_list& action) {
    int bin = 1;
    map<string, int> counts;
    data_list_cit action_it = action.begin();
    while (action_it != action.end()) {
        for (int joint = 1; joint <= 8; joint++) {
            labeled_action_list bins = get_bin(joint, bin, actions);
            string classification = bin_classification(*action_it++, bins);
            counts[classification]++;
        }

        bin++;
    }

    return get_max_in_map(counts);
}

bool Dataset::is_num(char c) {
    return isdigit(c) || c == '.' || c == 'e' || c== '+' || c == '-';
}

Dataset::action_list Dataset::build_bin_list(const vector<double>& values) {
    action_list data_points;

    // Looping through all the data points
    for (int i = 0; i < values.size(); i += 3) {
        data_point c;
        c.vel = values[i];
        c.pos = values[i + 1];
        c.eff = values[i + 2];
        data_points.push_back(c);
    }

    return data_points;
}

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

double Dataset::get_dist(const Dataset::data_point& data,
        const Dataset::data_point& action) {
    double delta_vel = pow(action.vel - data.vel, 2);
    double delta_pos = pow(action.pos - data.pos, 2);
    double delta_eff = pow(action.eff - data.eff, 2);

    return sqrt(delta_vel + delta_pos + delta_eff);
}

string Dataset::bin_classification(const data_point& point,
        const labeled_action_list& bins) {
    string closest_str = "";
    double closest_dist = 99999999999999.0;

    for (l_data_list_cit it = bins.begin(); it != bins.end(); it++) {
        double dist = get_dist(point, it->data);
        if (dist < closest_dist) {
            closest_dist = dist;
            closest_str = it->label;
        }
    }

    return closest_str;
}

Dataset::labeled_action_list Dataset::get_bin(int joint, int bin,
        const action_set& set) {
    labeled_action_list result;
    for (data_group_cit set_it = set.begin(); set_it != set.end(); set_it++) {
        action_list current_list = set_it->data;
        data_list_cit list_it = current_list.begin();

        // Moving list_it down to the appropriate bin
        for (int i = 1; i < bin * 8; i++) {
            list_it++;
        }

        // Moving list_it down the appropriate joint in the bin
        for (int i = 1; i < joint; i++) {
            list_it++;
        }

        // Building the labeled_data_point
        labeled_data_point ldp;
        ldp.label = set_it->classification;
        ldp.data = *list_it;

        // Pushing back the labeled_data_point
        result.push_back(ldp);
    }

    return result;
}

string Dataset::get_max_in_map(const map<string, int>& counts) {
    int max = 0;
    string max_key;

    for (map<string, int>::const_iterator it = counts.begin();
            it != counts.end(); it++) {
        if (it->second > max) {
            max = it->second;
            max_key = it->first;
        }
    }

    return max_key;
}

void Dataset::print_list(const action& ac) {
    // Looping through list and printing each data point
    for (data_list_cit it = ac.data.begin(); it != ac.data.end(); it++) {
        cout << it->vel << " " << it->pos << " " << it->eff << " ";
    }

    // Printing the classification
    cout << ac.classification << endl;
}
