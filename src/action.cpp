
        /**
         * Calculates the distance between two joint_states
         * Returns the distance
         */
        double joint_dist(const joint_state& data, const joint_state& recorded) {
                double dvel = pow(recorded.vel - data.vel, 2);
                double dpos = pow(recorded.pos - data.pos, 2);
                double deff = pow(recorded.eff - data.eff, 2);
            
                return sqrt(dvel + dpos + deff);
        }

        /**
         * Calculates the euclidean distance between two Points
         * Returns the distance
         */
        double euclidean_dist(const geometry_msgs::Point& data,                                             const geometry_msgs::Point& action) {
             
		double dx = pow(action.position.x - data.position.x, 2);
		double dy = pow(action.position.y - data.position.y, 2);
		double dz = pow(action.position.z - data.position.z, 2);
		double dist = sqrt(dx + dy + dz);

		// Returning the sum of the distances
		return dist;
        }

        /**
         * Calculates the distance between two Quaternions
         * Returns the distance
         */
        double quarterion_dist(const geometry_msgs::Quaternion,                                               const geometry_msgs::Quaternion) {


                Eigen::Vector4f dv;
 	        dv[0] = d.w; dv[1] = d.x; dv[2] = d.y; dv[3] = d.z;
	        Eigen::Matrix<float, 3,4> inv;
	        inv(0,0) = -c.x; inv(0,1) = c.w; inv(0,2) = -c.z; inv(0,3) = c.y;
  	        inv(1,0) = -c.y; inv(1,1) = c.z; inv(1,2) = -c.w; inv(1,3) = -c.x;
	        inv(2,0) = -c.z; inv(2,1) = -c.y; inv(2,2) = -c.x; inv(2,3) = c.w;

		Eigen::Vector3f m = inv * dv * -2.0;
		return m.norm();
        }





