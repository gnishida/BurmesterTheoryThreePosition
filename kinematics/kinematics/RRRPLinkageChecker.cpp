﻿#include "RRRPLinkageChecker.h"
#include "KinematicUtils.h"
#include "Kinematics.h"
#include "PinJoint.h"
#include "SliderHinge.h"
#include "BoundingBox.h"

namespace kinematics {

	/**
	* Calculate solutions of RRRP linkage given three poses.
	*
	* @param poses			three poses
	* @param solutions1	the output solutions for the driving crank, each of which contains a pair of the center point and the circle point
	* @param solutions2	the output solutions for the follower, each of which contains a pair of the fixed point and the slider point
	*/
	void calculateSolutionOfRRRPLinkageForThreePoses(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, int num_samples, double sigma, std::vector<Solution>& solutions1, std::vector<Solution>& solutions2) {
		solutions1.clear();
		solutions2.clear();

		srand(0);

		// convert the coordinates of the valid regions to the local coordinate system of the first pose
		glm::dmat3x3 inv_pose0 = glm::inverse(poses[0]);
		std::vector<glm::dvec2> valid_region(linkage_region_pts.size());
		for (int i = 0; i < linkage_region_pts.size(); i++) {
			valid_region[i] = glm::dvec2(inv_pose0 * glm::dvec3(linkage_region_pts[i], 1));
		}

		// calculate the bounding boxes of the valid regions
		BBox bbox = boundingBox(valid_region);

		// calculate the solutions for the driving crank
		int cnt = 0;
		for (int iter = 0; iter < num_samples * 100 && cnt < num_samples; iter++) {
			// sample a point within the valid region as the local coordinate of a circle point
			glm::dvec2 a(genRand(bbox.minPt.x, bbox.maxPt.x), genRand(bbox.minPt.y, bbox.maxPt.y));

			// if the sampled point is outside the valid region, discard it.
			if (!withinPolygon(valid_region, a)) continue;

			// perturbe the poses a little
			// HACK: 本来なら、bodyの座標を関数に渡し、関数側でpertubeしてからposeを計算すべきか？
			//       とりあえず、回転はperturbしていない。
			std::vector<glm::dmat3x3> perturbed_poses = poses;
			double pose_error = 0.0;
			for (int i = 1; i < poses.size() - 1; i++) {
				double e1 = genNormal(0, sigma);
				perturbed_poses[i][2][0] += e1;
				double e2 = genNormal(0, sigma);
				perturbed_poses[i][2][1] += e2;
				pose_error += e1 * e1 + e2 * e2;
			}

			glm::dvec2 A1(perturbed_poses[0] * glm::dvec3(a, 1));
			glm::dvec2 A2(perturbed_poses[1] * glm::dvec3(a, 1));
			glm::dvec2 A3(perturbed_poses[2] * glm::dvec3(a, 1));

			try {
				glm::dvec2 A0 = circleCenterFromThreePoints(A1, A2, A3);

				// if the center point is outside the valid region, discard it.
				if (!withinPolygon(linkage_region_pts, A0)) continue;

				solutions1.push_back(Solution(A0, A1, pose_error));
				cnt++;
			}
			catch (char* ex) {
			}
		}

		// calculate the solutions for the follower
		cnt = 0;
		for (int iter = 0; iter < num_samples * 100 && cnt < num_samples; iter++) {
			// sample a point within the valid region as the local coordinate of a circle point
			glm::dvec2 a(genRand(bbox.minPt.x, bbox.maxPt.x), genRand(bbox.minPt.y, bbox.maxPt.y));

			// if the sampled point is outside the valid region, discard it.
			if (!withinPolygon(valid_region, a)) continue;

			glm::dvec2 A1(poses[0] * glm::dvec3(a, 1));
			glm::dvec2 A2(poses[1] * glm::dvec3(a, 1));
			glm::dvec2 A3(poses[2] * glm::dvec3(a, 1));

			glm::dvec2 v1 = A2 - A1;
			double l1 = glm::length(v1);
			v1 /= l1;
			glm::dvec2 v2 = A3 - A1;
			double l2 = glm::length(v2);
			v2 /= l2;

			// check the order of A1, A2, and A3, and the collinearity of A1, A2, and A3
			if (glm::dot(v1, v2) > 0 && l2 > l1 &&  abs(crossProduct(v1, v2)) < 0.01) {
				for (int i = 0; i < 100; i++) {
					glm::dvec2 A0 = A1 + v1 * (double)(i - 50);
					if (A0 == A1) continue;

					// if the center point is outside the valid region, discard it.
					if (!withinPolygon(linkage_region_pts, A0)) continue;

					solutions2.push_back(Solution(A0, A1, 0));

					cnt++;
				}
			}
		}
	}

	std::vector<glm::dvec2> findBestSolutionOfRRRPLinkage(const std::vector<glm::dmat3x3>& poses, const std::vector<Solution>& solutions1, const std::vector<Solution>& solutions2, const std::vector<std::vector<glm::dvec2>>& fixed_body_pts, const std::vector<glm::dvec2>& body_pts, bool rotatable_crank, bool avoid_branch_defect, double min_link_length, double pose_error_weight, double smoothness_error) {
		time_t start = clock();

		std::vector<std::vector<Solution>> candidates;

		for (int i = 0; i < solutions1.size(); i++) {
			for (int j = 0; j < solutions2.size(); j++) {
				// check the length of the link
				if (glm::length(solutions1[i].fixed_point - solutions2[j].fixed_point) < min_link_length) continue;
				if (glm::length(solutions1[i].moving_point - solutions2[j].moving_point) < min_link_length) continue;

				if (rotatable_crank && checkRotatableCrankDefectForRRRPLinkage(solutions1[i].fixed_point, solutions2[j].fixed_point, solutions1[i].moving_point, solutions2[j].moving_point)) continue;
				if (avoid_branch_defect && checkBranchDefectForRRRPLinkage(poses, solutions1[i].fixed_point, solutions2[j].fixed_point, solutions1[i].moving_point, solutions2[j].moving_point)) continue;
				if (checkCircuitDefectForRRRPLinkage(poses, solutions1[i].fixed_point, solutions2[j].fixed_point, solutions1[i].moving_point, solutions2[j].moving_point)) continue;

				// collision check
				if (checkCollisionForRRRPLinkage(poses, solutions1[i].fixed_point, solutions2[j].fixed_point, solutions1[i].moving_point, solutions2[j].moving_point, fixed_body_pts, body_pts)) continue;

				candidates.push_back({ solutions1[i], solutions2[j] });
			}
		}

		// select the best solution based on the trajectory
		if (candidates.size() > 0) {
			time_t end = clock();
			std::cout << "Elapsed: " << (double)(end - start) / CLOCKS_PER_SEC << " sec for " << solutions1.size() << " x " << solutions2.size() << " solutions." << std::endl;
			start = clock();

			double min_length = std::numeric_limits<double>::max();
			int best = -1;
			for (int i = 0; i < candidates.size(); i++) {
				double length = lengthOfTrajectoryForRRRPLinkage(poses, candidates[i][0].fixed_point, candidates[i][1].fixed_point, candidates[i][0].moving_point, candidates[i][1].moving_point, body_pts);
				if (length < min_length) {
					min_length = length;
					best = i;
				}
			}

			end = clock();
			std::cout << "Elapsed: " << (double)(end - start) / CLOCKS_PER_SEC << " sec for " << candidates.size() << " candidates." << std::endl;
			std::cout << best << " th candidate was selected." << std::endl;

			return{ candidates[best][0].fixed_point, candidates[best][1].fixed_point, candidates[best][0].moving_point, candidates[best][1].moving_point };
		}
		else {
			time_t end = clock();
			std::cout << "Elapsed: " << (double)(end - start) / CLOCKS_PER_SEC << " sec for " << solutions1.size() << " x " << solutions2.size() << " solutions." << std::endl;
			std::cout << "No solution was found." << std::endl;

			std::vector<glm::dvec2> ans(4);
			ans[0] = glm::dvec2(0, 0);
			ans[1] = glm::dvec2(2, 0);
			ans[2] = glm::dvec2(0, 2);
			ans[3] = glm::dvec2(2, 2);
			return ans;
		}
	}

	/**
	* Return the RRRP linkage type.
	*
	* 0 -- rotatable crank
	* 1 -- 0-rocker
	* 2 -- pi-rocker
	* 3 -- rocker
	*/
	int getRRRPType(const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3) {
		// obtain the vectors, u (x axis) and v (y axis)
		glm::dvec2 u = p3 - p1;
		u /= glm::length(u);

		glm::dvec2 v(-u.y, u.x);
		if (glm::dot(p0 - p1, v) < 0) {
			u = -u;
			v = -v;
		}

		// calculate each length
		double e = glm::dot(p0 - p1, v);
		double r = glm::length(p2 - p0);
		double l = glm::length(p3 - p2);

		// calculate S1 and S2
		double S1 = l - r + e;
		double S2 = l - r - e;

		// judge the type of the RRRP linkage
		if (S1 >= 0 && S2 >= 0) return 0;
		else if (S1 >= 0 && S2 < 0) {
			// HACK to differentiate 0-rocker from pi-rocker
			if (v.y >= 0) return 1;
			else return 2;
		}
		//else if (S1 < 0 && S2 >= 0) return 2;
		else return 3;
	}

	/**
	* Check if the linkage has rotatable crank defect.
	* If the crank is not fully rotatable, true is returned.
	*/
	bool checkRotatableCrankDefectForRRRPLinkage(const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3) {
		int linkage_type = getRRRPType(p0, p1, p2, p3);

		if (linkage_type == 0) {
			return false;
		}
		else {
			return true;
		}
	}

	bool checkBranchDefectForRRRPLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3) {
		int type = getRRRPType(p0, p1, p2, p3);

		// rotatable crank always does not have a branch defect
		if (type == 0) return false;

		// obtain the vectors, u (x axis) and v (y axis)
		glm::dvec2 u = p3 - p1;
		u /= glm::length(u);

		glm::dvec2 v(-u.y, u.x);
		if (glm::dot(p0 - p1, v) < 0) {
			u = -u;
			v = -v;
		}

		int orig_sign = 1;

		// calculate the local coordinates of the circle points
		glm::dvec2 q2 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(p2, 1));
		glm::dvec2 q3 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(p3, 1));

		for (int i = 0; i < poses.size(); i++) {
			// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
			glm::dvec2 P2 = glm::dvec2(poses[i] * glm::dvec3(q2, 1));
			glm::dvec2 P3 = glm::dvec2(poses[i] * glm::dvec3(q3, 1));

			// calculate the sign of the dot product of L and u
			if (i == 0) {
				orig_sign = glm::dot(P3 - P2, u) >= 0 ? 1 : -1;
			}
			else {
				int sign = glm::dot(P3 - P2, u) >= 0 ? 1 : -1;
				if (sign != orig_sign) return true;
			}
		}

		return false;
	}

	bool checkCircuitDefectForRRRPLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3) {
		int type = getRRRPType(p0, p1, p2, p3);

		// 0-rocker and pi-rocker always do not have a branch defect
		if (type == 1 || type == 2) return false;

		// obtain the vectors, u (x axis) and v (y axis)
		glm::dvec2 u = p3 - p1;
		u /= glm::length(u);

		glm::dvec2 v(-u.y, u.x);
		if (glm::dot(p0 - p1, v) < 0) {
			u = -u;
			v = -v;
		}

		int orig_sign = 1;

		// calculate the local coordinates of the circle points
		glm::dvec2 q2 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(p2, 1));
		glm::dvec2 q3 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(p3, 1));

		for (int i = 0; i < poses.size(); i++) {
			// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
			glm::dvec2 P2 = glm::dvec2(poses[i] * glm::dvec3(q2, 1));
			glm::dvec2 P3 = glm::dvec2(poses[i] * glm::dvec3(q3, 1));

			// calculate the sign of the dot product of L and u
			if (i == 0) {
				if (type == 0) {
					orig_sign = glm::dot(P3 - P2, u) >= 0 ? 1 : -1;
				}
				else {
					orig_sign = glm::dot(P2 - p0, u) >= 0 ? 1 : -1;
				}
			}
			else {
				int sign;
				if (type == 0) {
					sign = glm::dot(P3 - P2, u) >= 0 ? 1 : -1;
				}
				else {
					sign = glm::dot(P2 - p0, u) >= 0 ? 1 : -1;
				}
				if (sign != orig_sign) return true;
			}
		}

		return false;
	}

	bool checkCollisionForRRRPLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, const std::vector<std::vector<glm::dvec2>>& fixed_body_pts, const std::vector<glm::dvec2>& body_pts) {
		kinematics::Kinematics kinematics(0.02);

		// construct a linkage
		kinematics.diagram.addJoint(boost::shared_ptr<PinJoint>(new PinJoint(0, true, p0)));
		kinematics.diagram.addJoint(boost::shared_ptr<PinJoint>(new PinJoint(1, true, p1)));
		kinematics.diagram.addJoint(boost::shared_ptr<PinJoint>(new PinJoint(2, false, p2)));
		kinematics.diagram.addJoint(boost::shared_ptr<SliderHinge>(new SliderHinge(3, false, p3)));
		kinematics.diagram.addLink(true, kinematics.diagram.joints[0], kinematics.diagram.joints[2]);
		kinematics.diagram.addLink(false, kinematics.diagram.joints[1], kinematics.diagram.joints[3]);
		kinematics.diagram.addLink(false, kinematics.diagram.joints[2], kinematics.diagram.joints[3]);

		// set the geometry
		for (int i = 0; i < fixed_body_pts.size(); i++) {
			kinematics.diagram.addBody(kinematics.diagram.joints[0], kinematics.diagram.joints[1], fixed_body_pts[i]);
		}
		kinematics.diagram.addBody(kinematics.diagram.joints[2], kinematics.diagram.joints[3], body_pts);

		kinematics.diagram.initialize();

		// calculate the rotational angle of the driving crank for 1st, 2nd, and last poses
		// i.e., angles[0] = first pose, angles[1] = second pose, angles[2] = last pose
		std::vector<double> angles(3);
		glm::dvec2 w(glm::inverse(poses[0]) * glm::dvec3(p2, 1));
		for (int i = 0; i < 2; i++) {
			glm::dvec2 W = glm::dvec2(poses[i] * glm::dvec3(w, 1));
			angles[i] = atan2(W.y - p0.y, W.x - p0.x);
		}
		{
			glm::dvec2 W = glm::dvec2(poses.back() * glm::dvec3(w, 1));
			angles[2] = atan2(W.y - p0.y, W.x - p0.x);
		}

		// order the angles based on their signs
		int type = 0;
		if (angles[0] < 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] >= angles[1]) {
			type = 1;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] < angles[2]) {
			type = 2;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] >= angles[2]) {
			type = 3;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] >= 0 && angles[1] >= angles[2]) {
			type = 4;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] < 0 && angles[1] < angles[2]) {
			type = 5;
			angles[1] += M_PI * 2;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] < angles[2]) {
			type = 6;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] >= angles[2]) {
			type = 7;
			angles[1] += M_PI * 2;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] < angles[1]) {
			type = 8;
			angles[2] += M_PI * 2;
		}

		if (angles[2] < angles[0]) {
			kinematics.invertSpeed();
		}

		// initialize the visited flag
		std::vector<bool> visited(poses.size(), false);
		visited[0] = true;
		int unvisited = 2;

		// run forward until collision is deteted or all the poses are reached
		while (true) {
			try {
				kinematics.stepForward(true, false);
			}
			catch (char* ex) {
				// if only some of the poses are reached before collision, the collision is detected.
				kinematics.clear();
				return true;
			}

			// calculate the angle of the driving crank
			double angle = atan2(kinematics.diagram.joints[2]->pos.y - p0.y, kinematics.diagram.joints[2]->pos.x - p0.x);

			// convert the sign of the angle
			if (type == 1 && angle > 0) {
				angle -= M_PI * 2;
			}
			else if (type == 2 && angle > angles[0]) {
				angle -= M_PI * 2;
			}
			else if (type == 3 && angle < angles[0]) {
				angle += M_PI * 2;
			}
			else if (type == 4 && angle > 0) {
				angle -= M_PI * 2;
			}
			else if (type == 5 && angle < 0) {
				angle += M_PI * 2;
			}
			else if (type == 6 && angle > angles[0]) {
				angle -= M_PI * 2;
			}
			else if (type == 7 && angle < angles[0]) {
				angle += M_PI * 2;
			}
			else if (type == 8 && angle < 0) {
				angle += M_PI * 2;
			}

			// check if the poses are reached
			for (int i = 0; i < angles.size(); i++) {
				if (visited[i]) continue;

				if (angles[2] >= angles[0]) {
					if (angle >= angles[i]) {
						visited[i] = true;
						unvisited--;
					}
				}
				else {
					if (angle <= angles[i]) {
						visited[i] = true;
						unvisited--;
					}
				}
			}

			// if all the poses are reached without collision, no collision is detected.
			if (unvisited == 0) {
				kinematics.clear();
				return false;
			}
		}

		kinematics.clear();
		return false;
	}

	double lengthOfTrajectoryForRRRPLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, const std::vector<glm::dvec2>& body_pts) {
		kinematics::Kinematics kinematics(0.1);

		// construct a linkage
		kinematics.diagram.addJoint(boost::shared_ptr<PinJoint>(new PinJoint(0, true, p0)));
		kinematics.diagram.addJoint(boost::shared_ptr<PinJoint>(new PinJoint(1, true, p1)));
		kinematics.diagram.addJoint(boost::shared_ptr<PinJoint>(new PinJoint(2, false, p2)));
		kinematics.diagram.addJoint(boost::shared_ptr<SliderHinge>(new SliderHinge(3, false, p3)));
		kinematics.diagram.addLink(true, kinematics.diagram.joints[0], kinematics.diagram.joints[2]);
		kinematics.diagram.addLink(false, kinematics.diagram.joints[1], kinematics.diagram.joints[3]);
		kinematics.diagram.addLink(false, kinematics.diagram.joints[2], kinematics.diagram.joints[3]);

		// set the geometry
		kinematics.diagram.addBody(kinematics.diagram.joints[2], kinematics.diagram.joints[3], body_pts);

		kinematics.diagram.initialize();

		// initialize the trajectory of the moving body
		std::vector<glm::dvec2> prev_body_pts = body_pts;
		double length_of_trajectory = 0.0;

		// calculate the rotational angle of the driving crank for 1st, 2nd, and last poses
		// i.e., angles[0] = first pose, angles[1] = second pose, angles[2] = last pose
		std::vector<double> angles(3);
		glm::dvec2 w(glm::inverse(poses[0]) * glm::dvec3(p2, 1));
		for (int i = 0; i < 2; i++) {
			glm::dvec2 W = glm::dvec2(poses[i] * glm::dvec3(w, 1));
			angles[i] = atan2(W.y - p0.y, W.x - p0.x);
		}
		{
			glm::dvec2 W = glm::dvec2(poses.back() * glm::dvec3(w, 1));
			angles[2] = atan2(W.y - p0.y, W.x - p0.x);
		}

		// order the angles based on their signs
		int type = 0;
		if (angles[0] < 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] >= angles[1]) {
			type = 1;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] < angles[2]) {
			type = 2;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] >= angles[2]) {
			type = 3;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] >= 0 && angles[1] >= angles[2]) {
			type = 4;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] < 0 && angles[1] < angles[2]) {
			type = 5;
			angles[1] += M_PI * 2;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] < angles[2]) {
			type = 6;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] >= angles[2]) {
			type = 7;
			angles[1] += M_PI * 2;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] < angles[1]) {
			type = 8;
			angles[2] += M_PI * 2;
		}

		if (angles[2] < angles[0]) {
			kinematics.invertSpeed();
		}

		// initialize the visited flag
		std::vector<bool> visited(poses.size(), false);
		visited[0] = true;
		int unvisited = 2;

		// run forward until collision is deteted or all the poses are reached
		while (true) {
			try {
				kinematics.stepForward(true, false);
			}
			catch (char* ex) {
				// if only some of the poses are reached before collision, the collision is detected.
				kinematics.clear();
				return length_of_trajectory;
			}

			// calculate the angle of the driving crank
			double angle = atan2(kinematics.diagram.joints[2]->pos.y - p0.y, kinematics.diagram.joints[2]->pos.x - p0.x);

			// update the lengths of the trajectory of the moving body
			std::vector<glm::dvec2> next_body_pts = kinematics.diagram.bodies[0]->getActualPoints();
			for (int i = 0; i < next_body_pts.size(); i++) {
				double length = glm::length(next_body_pts[i] - prev_body_pts[i]);
				length_of_trajectory += length;
				prev_body_pts[i] = next_body_pts[i];
			}

			// convert the sign of the angle
			if (type == 1 && angle > 0) {
				angle -= M_PI * 2;
			}
			else if (type == 2 && angle > angles[0]) {
				angle -= M_PI * 2;
			}
			else if (type == 3 && angle < angles[0]) {
				angle += M_PI * 2;
			}
			else if (type == 4 && angle > 0) {
				angle -= M_PI * 2;
			}
			else if (type == 5 && angle < 0) {
				angle += M_PI * 2;
			}
			else if (type == 6 && angle > angles[0]) {
				angle -= M_PI * 2;
			}
			else if (type == 7 && angle < angles[0]) {
				angle += M_PI * 2;
			}
			else if (type == 8 && angle < 0) {
				angle += M_PI * 2;
			}

			// check if the poses are reached
			for (int i = 0; i < angles.size(); i++) {
				if (visited[i]) continue;

				if (angles[2] >= angles[0]) {
					if (angle >= angles[i]) {
						visited[i] = true;
						unvisited--;
					}
				}
				else {
					if (angle <= angles[i]) {
						visited[i] = true;
						unvisited--;
					}
				}
			}

			// if all the poses are reached without collision, no collision is detected.
			if (unvisited == 0) {
				kinematics.clear();
				return length_of_trajectory;
			}
		}

		kinematics.clear();
		return length_of_trajectory;
	}

}