#pragma once

#include <vector>
#include <glm/glm.hpp>

namespace kinematics {

	void calculateSolutionOfRRRPLinkageForThreePoses(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, int num_samples, double sigma, std::vector<std::pair<glm::dvec2, glm::dvec2>>& solutions1, std::vector<std::pair<glm::dvec2, glm::dvec2>>& solutions2);
	std::vector<glm::dvec2> findBestSolutionOfRRRPLinkage(const std::vector<glm::dmat3x3>& poses, const std::vector<std::pair<glm::dvec2, glm::dvec2>>& solutions1, const std::vector<std::pair<glm::dvec2, glm::dvec2>>& solutions2, const std::vector<std::vector<glm::dvec2>>& fixed_body_pts, const std::vector<glm::dvec2>& body_pts, bool rotatable_crank, bool avoid_branch_defect, double min_link_length);

	int getRRRPType(const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3);
	bool checkRotatableCrankDefectForRRRPLinkage(const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3);
	bool checkBranchDefectForRRRPLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3);
	bool checkCircuitDefectForRRRPLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3);
	bool checkCollisionForRRRPLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, const std::vector<std::vector<glm::dvec2>>& fixed_body_pts, const std::vector<glm::dvec2>& body_pts);
	double lengthOfTrajectoryForRRRPLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, const std::vector<glm::dvec2>& body_pts);

}