#include "Solution.h"

namespace kinematics {

	Solution::Solution(const glm::dvec2& fixed_point, const glm::dvec2& moving_point, double pose_error) {
		this->fixed_point = fixed_point;
		this->moving_point = moving_point;
		this->pose_error = pose_error;
	}

}