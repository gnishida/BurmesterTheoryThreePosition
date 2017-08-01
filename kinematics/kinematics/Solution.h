#pragma once

#include <glm/glm.hpp>

namespace kinematics {

	class Solution {
	public:
		glm::dvec2 fixed_point;
		glm::dvec2 moving_point;
		double pose_error;

	public:
		Solution(const glm::dvec2& fixed_point, const glm::dvec2& moving_point, double pose_error);
	};

}