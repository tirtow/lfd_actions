#include "dataset.h"
#include <cmath>

using std::ifstream;
using std::cout;
using std::endl;
using std::list;
using std::vector;
using std::string;
using std::map;

bool Dataset::is_num(char c) {
    return isdigit(c) || c == '.';
}

list<Dataset::data_point> Dataset::build_bin_list(const vector<float>& values) {
    list<Dataset::data_point> data_points;

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

string Dataset::split_line(const string& line, vector<float>& values) {
    // Iterating over entire line
    string::const_iterator begin = line.begin();
    while (is_num(*begin)) {
        // Moving end down until at next non-digit character
        string::const_iterator end = begin;
        while (is_num(*end)) {
            end++;
        }

        // Building string from iterators and pushing float value to vector
        string val(begin, end);
        values.push_back(atof(val.c_str()));

        // Moving begin to one past end
        begin = end + 1;
    }

    // Getting the classification from the remaining characters
    string classification(begin, line.end());

    return classification;
}

void Dataset::print_list(const string& classification,
        const list<Dataset::data_point>& data) {
    // Looping through list and printing each data point
    for (data_list_cit it = data.begin(); it != data.end(); it++) {
        cout << it->vel << " " << it->pos << " " << it->eff << " ";
    }

    // Printing the classification
    cout << classification << endl;
}

Dataset::Dataset(ifstream& is) {
    // Looping while not at end of file
    while (is) {
        // Getting the line
        string line;
        getline(is, line);

        // Splitting the line into a vector of floats
        vector<float> values;
        string classification = split_line(line, values);

        // Building the data_points list
        list<data_point> data_points = build_bin_list(values);

        // Pushing to appropriate list
        if (classification == "lift") {
            lifts.push_back(data_points);
        } else if (classification == "sweep") {
            sweeps.push_back(data_points);
        }
    }
}

string Dataset::guess_classification(const action_list& action) {
    int bin = 1;
    map<string, int> counts;
    for (data_list_cit action_it = action.begin();
            action_it != action.end(); action_it++) {
        for (int joint = 1; joint <= 8; joint++) {
            action_list lift_bins = get_bin(joint, bin, lifts);
            action_list sweep_bins = get_bin(joint, bin, sweeps);
            string classification = bin_classification(action, lift_bins, sweep_bins);
            counts[classification]++;
        }

        bin++;
    }
}

double Dataset::get_dist(const Dataset::data_point& data,
        const Dataset::data_point& action) {
    double delta_vel = pow(action.vel - data.vel, 2);
    double delta_pos = pow(action.pos - data.pos, 2);
    double delta_eff = pow(action.eff - data.eff, 2);

    return sqrt(delta_vel + delta_pos + delta_eff);
}

Dataset::action_list Dataset::get_bin(int joint, int bin, const action_set& set) {
    action_list result;
    for (data_group_cit set_it = set.begin(); set_it != set.end(); set_it++) {
        action_list current_list = *set_it;
        data_list_cit list_it = current_list.begin();

        // Moving list_it down to the appropriate bin
        for (int i = 1; i < bin * 24; i++) {
            list_it++;
        }

        // Moving list_it down the appropriate joint in the bin
        for (int i = 1; i < joint * 3; i++) {
            list_it++;
        }

        // Pushing back the data_point
        result.push_back(*list_it);
    }

    return result;
}

string get_max_in_map(const map<string, int>& counts) {
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

=======
string bin_classification(const data_point& point, const list<data_point>& liftPoints
   , const list<data_points>& sweepPoints){

   double liftCmp = 99999999999999.0;
   double sweepCmp = 99999999999999.0;

    for(data_list_cit it = liftPoints.begin(); it != liftPoints.end(); it++) {
        double tempLiftCmp = get_dist(point, *it);

        if( liftCmp > tempLiftCmp )
        {
          liftCmp = tempLiftCmp;
        }
        
    }

    for(data_list_cit it = sweepPoints.begin(); it != sweepPoints.end(); it++) {
       double tempSweepCmp = get_dist(point, *it);
   
       if( sweepCmp > tempSweepCmp )
       {
         sweepCmp = tempLiftCmp;
       }

    }

    // Compare the values of lifts and sweeps to the data point of the unknown action
    // return the classification of the one with the smaller value


    if( liftCmp > sweepCmp)
    {
       return "sweep";
    }
    else
    {
       return "lift";
    }
}
>>>>>>> dc1f20135147bbe6e89a698938928d78b0be4fc9
void Dataset::print_dataset() {
    // Printing all the data for lifts
    for (data_group_cit it = lifts.begin(); it != lifts.end(); it++) {
        print_list("lift", *it);
    }

    // Printing all the data for sweeps
    for (data_group_cit it = sweeps.begin(); it != sweeps.end(); it++) {
        print_list("sweep", *it);
    }
}
