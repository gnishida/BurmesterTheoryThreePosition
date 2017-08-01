#pragma once

#include <vector>
#include <glm/glm.hpp>
#include "Solution.h"

namespace kinematics {

	void calculateSolutionOf4RLinkage(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, int num_samples, double sigma, std::vector<Solution>& solutions1, std::vector<Solution>& solutions2);
	void calculateSolutionOf4RLinkageForThreePoses(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, int num_samples, double sigma, std::vector<Solution>& solutions1, std::vector<Solution>& solutions2);
	std::vector<glm::dvec2> findBestSolutionOf4RLinkage(const std::vector<glm::dmat3x3>& poses, const std::vector<Solution>& solutions1, const std::vector<Solution> &solutions2, const std::vector<std::vector<glm::dvec2>>& fixed_body_pts, const std::vector<glm::dvec2>& body_pts, bool rotatable_crank, bool avoid_branch_defect, double min_link_length, double pose_error_weight, double smoothness_error);

	int getGrashofType(const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3);
	bool checkFolding(const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3);
	bool checkRotatableCrankDefectFor4RLinkage(const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3);
	bool checkOrderDefectFor4RLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, bool debug = false);
	bool checkBranchDefectFor4RLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, bool debug = false);
	bool checkCircuitDefectFor4RLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, bool debug = false);
	bool checkCollisionFor4RLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, const std::vector<std::vector<glm::dvec2>>& fixed_body_pts, const std::vector<glm::dvec2>& body_pts);
	double lengthOfTrajectoryFor4RLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, const std::vector<glm::dvec2>& body_pts);

}