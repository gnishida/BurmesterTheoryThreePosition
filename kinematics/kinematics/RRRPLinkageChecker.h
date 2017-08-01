#pragma once

#include <vector>
#include <glm/glm.hpp>
#include "Solution.h"
#include "BBox.h"

namespace kinematics {

	void calculateSolutionOfRRRPLinkageForThreePoses(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, int num_samples, const std::vector<std::vector<glm::dvec2>>& fixed_body_pts, const std::vector<glm::dvec2>& body_pts, double sigma, bool rotatable_crank, bool avoid_branch_defect, double min_link_length, std::vector<Solution>& solutions);
	bool sampleLinkForRRRPLinkage(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_region_pts_local, const BBox& bbox, glm::dvec2& A0, glm::dvec2& A1);
	bool sampleSliderForRRRPLinkage(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_region_pts_local, const BBox& bbox, glm::dvec2& A0, glm::dvec2& A1);
	Solution findBestSolutionOfRRRPLinkage(const std::vector<glm::dmat3x3>& poses, const std::vector<Solution>& solutions, const std::vector<std::vector<glm::dvec2>>& fixed_body_pts, const std::vector<glm::dvec2>& body_pts, double pose_error_weight, double smoothness_weight);

	int getRRRPType(const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3);
	bool checkRotatableCrankDefectForRRRPLinkage(const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3);
	bool checkBranchDefectForRRRPLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3);
	bool checkCircuitDefectForRRRPLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3);
	bool checkCollisionForRRRPLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, const std::vector<std::vector<glm::dvec2>>& fixed_body_pts, const std::vector<glm::dvec2>& body_pts);
	double tortuosityOfTrajectoryForRRRPLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, const std::vector<glm::dvec2>& body_pts);

}