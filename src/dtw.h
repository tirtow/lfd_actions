#ifndef GUARD_dtw_h
#define GUARD_dtw_h

#include "action.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

class DTW {
    public:
        /**
         * Calculates the minimum difference between two actions
         * using dynamic time warping.
         * Returns the minimum difference between the actions
         */
        static double min_diff(const Action&, const Action&);

    private:
        /**
         * Generates the difference between every point in both Actions
         * using Action::get_dist and stores them in the 2D double array
         */
        static void get_diffs(const Action&, const Action&, double**);

        /**
         * Gets the minium value between three doubles
         * Returns the minimum value
         */
        static double min(double, double, double);

        /**
         * Calculates the distance between two Poses
         * Returns the distance
         */
        static double distance(const geometry_msgs::Pose&, const geometry_msgs::Pose&);

        /**
         * Calculates the distance between two points using euclidean distance
         * Returns the distance
         */
        static double position_distance(const geometry_msgs::Point&, const geometry_msgs::Point&);

        /**
         * Calculates the distance between two quaternions
         * Returns the distance
         */
        static double quaternion_distance(const geometry_msgs::Quaternion&, const geometry_msgs::Quaternion&);
};

#endif
