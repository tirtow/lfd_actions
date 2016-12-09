#ifndef GUARD_dtw_h
#define GUARD_dtw_h

#include "action.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/JointState.h>

/**
 * Class to calculate the minimum difference between two Actions
 * using dynamic time warping
 */
class DTW {
    public:
        /**
         * Calculates the minimum difference between two actions
         * using dynamic time warping.
         */
        static double min_diff(const Action&, const Action&);

    private:
        /**
         * Gets the minium value between three doubles
         */
        static double min(double, double, double);

        /**
         * Calculates the distance between two Poses
         */
        static double distance(const geometry_msgs::Pose&,
                const geometry_msgs::Pose&, const sensor_msgs::JointState&,
                const sensor_msgs::JointState&);

        /**
         * Calculates the distance between two points using euclidean distance
         */
        static double position_distance(const geometry_msgs::Point&,
                const geometry_msgs::Point&);

        /**
         * Calculates the distance between two quaternions
         */
        static double quaternion_distance(const geometry_msgs::Quaternion&,
                const geometry_msgs::Quaternion&);

        /**
         * Calculates the distance between the two joint states
         */
        static double joint_distance(const sensor_msgs::JointState&,
                const sensor_msgs::JointState&);

        /**
         * Calculates the distance between two positions from the joint state
         */
        static double pos_dist(double, double);

        /**
         * Calculates the distance between two velocities from the joint state
         */
        static double vel_dist(double, double);

        /**
         * Reduces a radian such that it falls in the range [-PI, PI]
         */
        static double reduce_radian(double);
};

#endif
