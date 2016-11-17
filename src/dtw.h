#ifndef GUARD_dtw_h
#define GUARD_dtw_h

#include "action.h"

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
};

#endif
