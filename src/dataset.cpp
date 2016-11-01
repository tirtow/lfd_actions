#include "dataset.h"
#include <cmath>

using std::ifstream;
using std::list;
using std::vector;
using std::string;

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
