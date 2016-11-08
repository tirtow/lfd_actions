#include "dataset.h"
#include <cmath>

using std::ifstream;
using std::cout;
using std::endl;
using std::list;
using std::vector;
using std::string;
using std::map;
using geometry_msgs::Pose;

Dataset::Dataset(ifstream& is, int k_val = 1) : k(k_val) {
    // Looping while not at end of file
    while (is) {
        // Getting the line
        string line;
        getline(is, line);

		if (line != "") {
        // Splitting the line into a vector of doubles
        vector<double> values;
        string classification = split_line(line, values);

/*
        // Building the data_points list
        action_list data_points = build_bin_list(values);

        // Building the action
        action ac;
        ac.classification = classification;
        ac.data = data_points;
        */

        list<Pose> data = build_bin_list_cart(values);
        action_cart ac_c;
        ac_c.classification = classification;
        ac_c.cartesian = data;
        

        //actions.push_back(ac);
        cart_actions.push_back(ac_c);
	}

    }
}

void Dataset::print_dataset() {
    for (cart_cit it = cart_actions.begin(); it != cart_actions.end(); it++) {
		for (pose_it pit = it->cartesian.begin(); pit != it->cartesian.end(); pit++) {
			cout << pit->position.x << " " << pit->position.y << " " << pit->position.z << " "
			     << pit->orientation.x << " " << pit->orientation.y << " " << pit->orientation.z << " " << pit->orientation.w << " ";
		}
		
		cout << it->classification << endl;
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

void Dataset::add(const action_cart& recorded) {
    cart_actions.push_back(recorded);
}

bool Dataset::is_num(char c) {
    return isdigit(c) || c == '.' || c == 'e' || c== '+' || c == '-';
}

Dataset::action_list Dataset::build_bin_list(const vector<double>& values) {
    action_list data_points;

    // Looping through all the data points
    for (int i = 0; i < values.size(); i += 10) {
        data_point c;
        c.vel = values[i];
        c.pos = values[i + 1];
        c.eff = values[i + 2];
        data_points.push_back(c);
    }

    return data_points;
}

list<Pose> Dataset::build_bin_list_cart(const vector<double>& values) {
    list<Pose> result;

    // Looping through all the data points
    for (int i = 0; i < values.size(); i += 7) {
        Pose p;
        p.position.x = values[i];
        p.position.y = values[i + 1];
        p.position.z = values[i + 2];
        p.orientation.x = values[i + 3];
        p.orientation.y = values[i + 4];
        p.orientation.z = values[i + 5];
        p.orientation.w = values[i + 6];
        result.push_back(p);
    }

    return result;
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

double Dataset::get_dist(const Pose& data, const Pose& action) {
    double d_pos_x = pow(action.position.x - data.position.x, 2);
    double d_pos_y = pow(action.position.y - data.position.y, 2);
    double d_pos_z = pow(action.position.z - data.position.z, 2);
    double pos_dist = sqrt(d_pos_x + d_pos_y + d_pos_z);

    double d_ori_x = pow(action.orientation.x - data.orientation.x, 2);
    double d_ori_y = pow(action.orientation.y - data.orientation.y, 2);
    double d_ori_z = pow(action.orientation.z - data.orientation.z, 2);
    double d_ori_w = pow(action.orientation.w - data.orientation.w, 2);
    double pos_orient = sqrt(d_ori_x + d_ori_y + d_ori_z + d_ori_w);

    return pos_dist + pos_orient;
}

string Dataset::guess_classification_cart(const list<Pose>& recorded) {
    //vector<double> closest_dist;
    //vector<string> closest_str;
    double min= 999999999999;
    string min_str = "none";

    // Looping through each action in the dataset
    for (cart_cit set_it = cart_actions.begin(); set_it != cart_actions.end(); set_it++) {
        action_cart current = *set_it;
        double dist_sum = 0;

		ROS_INFO("%s", current.classification.c_str());

        pose_it recorded_it = recorded.begin();
        // Summing up all the distances for this action
        for (pose_it it = current.cartesian.begin();
                it != current.cartesian.end(); it++) {
            dist_sum += get_dist(*recorded_it++, *it);
        }

        if (dist_sum < min) {
			min = dist_sum;
			min_str = current.classification;
		}
    }

    return min_str;

/*
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

    // Returning the string that occurs the most
    //return get_max_in_map(get_counts(closest_str));
    return closest_str[0];
*/
}

string Dataset::guess_classification_alt(const action_list& recorded) {
    vector<double> closest_dist;
    vector<string> closest_str;

    // Looping through each action in the dataset
    for (data_group_cit set_it = actions.begin(); set_it != actions.end(); set_it++) {
        action current = *set_it;
        double dist_sum = 0;

        data_list_cit recorded_it = recorded.begin();
        // Summing up all the distances for this action
        for (data_list_cit it = current.data.begin();
                it != current.data.end(); it++) {
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

    // Returning the string that occurs the most
    return get_max_in_map(get_counts(closest_str));
}

map<string, int> Dataset::get_counts(const vector<string>& values) {
    map<string, int> counts;
    for (vector<string>::const_iterator it = values.begin();
            it != values.end(); it++) {
        counts[*it]++;
    }

    return counts;
}

string Dataset::bin_classification(const data_point& point,
        const labeled_action_list& bins) {
    map<int, string> closest;
    vector<string> closest_str;
    vector<double> closest_dist;

    // Calculating the distance between each bin and getting the 3 nearest
    for (l_data_list_cit it = bins.begin(); it != bins.end(); it++) {
        double dist = get_dist(point, it->data);

        // Looping through closest data_points to see if need to place
        bool found = false;
        vector<double>::iterator dist_it = closest_dist.begin();
        vector<string>::iterator str_it = closest_str.begin();
        while (!found && dist_it != closest_dist.end()) {
            if (dist < *dist_it) {
                closest_dist.insert(dist_it, dist);
                closest_str.insert(str_it, it->label);
                found = true;
            }

            dist_it++;
            str_it++;
        }

        if (!found && closest_dist.size() < k) {
            // Have less than k elements, adding to end
            closest_dist.push_back(dist);
            closest_str.push_back(it->label);
        } else if (closest_dist.size() > k) {
            // Have more than k elements, deleting last one
            closest_dist.pop_back();
            closest_str.pop_back();
        }
    }

    // Tallying the occurances of each label
    map<string, int> counts;
    for (vector<string>::const_iterator it = closest_str.begin();
            it != closest_str.end(); it++) {
        counts[*it]++;
    }

    // Returning the label that occurs most often
    return get_max_in_map(counts);
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
