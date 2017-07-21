#pragma once

#include <vector>
#include <glm/glm.hpp>

namespace kinematics {

	void calculateSolutionOf4RLinkageForThreePoses(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, std::vector<std::pair<glm::dvec2, glm::dvec2>>& solutions1, std::vector<std::pair<glm::dvec2, glm::dvec2>>& solutions2);
	std::vector<glm::dvec2> findBestSolutionOf4RLinkage(const std::vector<glm::dmat3x3>& poses, const std::vector<std::pair<glm::dvec2, glm::dvec2>>& solutions1, const std::vector<std::pair<glm::dvec2, glm::dvec2>>& solutions2, const std::vector<std::vector<glm::dvec2>>& fixed_body_pts, const std::vector<glm::dvec2>& body_pts, bool avoidGrashofDefect, bool avoidBranchDefect, double min_link_length);

	int getGrashofType(const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3);
	bool checkGrashofDefect(const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3);
	bool checkOrderDefect(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3);
	bool checkBranchDefectFor4RLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3);
	bool checkCircuitDefectFor4RLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3);
	bool checkCollisionFor4RLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, const std::vector<std::vector<glm::dvec2>>& fixed_body_pts, const std::vector<glm::dvec2>& body_pts);

}