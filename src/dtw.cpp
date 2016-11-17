#include "dtw.h"

using geometry_msgs::Position;
using geometry_msgs::Pose;
using geometry_msgs::Quarternion;

static void DTW::get_diffs(const Action& x, const Action& y, double** diffs) {
    int row = 0;
    int col = 0;

    for (pose_list x_it = x.begin(); x_it != x.end(); x_it++) {
        for (pose_list y_it = y.begin(); y_it != y.end(); y_it++) {
            diffs[row][col] = distance(*x_it, *y_it);
        }
    }
}

static double DTW::min(double x, double y, double z) {
    if (x < y) {
        return (x < z) ? x : z;
    } else {
        return (y < z) ? y : z;
    }
}

static double DTW::distance(const Pose& x, const Pose& y) {
    return position_distance(x.position, y.position)
            + orientation_distance(x.orientation, y.orientation);
}
