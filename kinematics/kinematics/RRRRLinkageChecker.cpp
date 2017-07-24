#include "RRRRLinkageChecker.h"
#include "KinematicUtils.h"
#include "Kinematics.h"
#include "PinJoint.h"
#include "SliderHinge.h"
#include "BoundingBox.h"

namespace kinematics {

	/**
	* Calculate solutions of 4R linkage given three poses.
	*
	* @param poses			three poses
	* @param solutions1	the output solutions for the world coordinates of the driving crank at the first pose, each of which contains a pair of the center point and the circle point
	* @param solutions2	the output solutions for the world coordinates of the follower at the first pose, each of which contains a pair of the center point and the circle point
	*/
	void calculateSolutionOf4RLinkageForThreePoses(const std::vector<glm::dmat3x3>& poses, const std::vector<glm::dvec2>& linkage_region_pts, std::vector<std::pair<glm::dvec2, glm::dvec2>>& solutions1, std::vector<std::pair<glm::dvec2, glm::dvec2>>& solutions2) {
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

		int cnt = 0;
		for (int iter = 0; iter < 10000000 && cnt < 1000; iter++) {
			// sample a point within the valid region as the local coordinate of a circle point
			glm::dvec2 a(genRand(bbox.minPt.x, bbox.maxPt.x), genRand(bbox.minPt.y, bbox.maxPt.y));

			// if the sampled point is outside the valid region, discard it.
			if (!withinPolygon(valid_region, a)) continue;

			glm::dvec2 A1(poses[0] * glm::dvec3(a, 1));
			glm::dvec2 A2(poses[1] * glm::dvec3(a, 1));
			glm::dvec2 A3(poses[2] * glm::dvec3(a, 1));

			try {
				glm::dvec2 A0 = circleCenterFromThreePoints(A1, A2, A3);

				// if the center point is outside the valid region, discard it.
				if (!withinPolygon(linkage_region_pts, A0)) continue;

				solutions1.push_back(std::make_pair(A0, A1));
				solutions2.push_back(std::make_pair(A0, A1));
				cnt++;
			}
			catch (char* ex) {
			}
		}
	}

	std::vector<glm::dvec2> findBestSolutionOf4RLinkage(const std::vector<glm::dmat3x3>& poses, const std::vector<std::pair<glm::dvec2, glm::dvec2>>& solutions1, const std::vector<std::pair<glm::dvec2, glm::dvec2>>& solutions2, const std::vector<std::vector<glm::dvec2>>& fixed_body_pts, const std::vector<glm::dvec2>& body_pts, bool avoidGrashofDefect, bool avoidBranchDefect, double min_link_length) {
		time_t start = clock();

		std::vector<glm::dvec2> ans(4);

		for (int i = 0; i < solutions1.size(); i++) {
			for (int j = 0; j < solutions2.size(); j++) {
				// check the length of the link
				if (glm::length(solutions1[i].first - solutions2[j].first) < min_link_length) continue;
				if (glm::length(solutions1[i].second - solutions2[j].second) < min_link_length) continue;

				if (avoidGrashofDefect && checkGrashofDefect(solutions1[i].first, solutions2[j].first, solutions1[i].second, solutions2[j].second)) continue;
				if (avoidBranchDefect && checkBranchDefectFor4RLinkage(poses, solutions1[i].first, solutions2[j].first, solutions1[i].second, solutions2[j].second)) continue;
				if (checkCircuitDefectFor4RLinkage(poses, solutions1[i].first, solutions2[j].first, solutions1[i].second, solutions2[j].second)) continue;

				// collision check
				if (checkCollisionFor4RLinkage(poses, solutions1[i].first, solutions2[j].first, solutions1[i].second, solutions2[j].second, fixed_body_pts, body_pts)) continue;

				ans[0] = solutions1[i].first;
				ans[1] = solutions2[j].first;
				ans[2] = solutions1[i].second;
				ans[3] = solutions2[j].second;

				time_t end = clock();
				std::cout << "Elapsed: " << (double)(end - start) / CLOCKS_PER_SEC << " sec for " << solutions1.size() << " x " << solutions2.size() << " solutions." << std::endl;

				return ans;
			}
		}

		time_t end = clock();
		std::cout << "Elapsed: " << (double)(end - start) / CLOCKS_PER_SEC << " sec for " << solutions1.size() << " x " << solutions2.size() << " solutions." << std::endl;

		std::cout << "No solution was found." << std::endl;
		ans[0] = glm::dvec2(0, 0);
		ans[1] = glm::dvec2(2, 0);
		ans[2] = glm::dvec2(0, 2);
		ans[3] = glm::dvec2(2, 2);
		return ans;
	}

	/**
	* Return the Grashof type.
	*
	* 0 -- Grashof (Drag-link)
	* 1 -- Grashof (Crank-rocker)
	* 2 -- Grashof (Rocker-crank)
	* 3 -- Grashof (Double-rocker)
	* 4 -- Non-Grashof (0-0 Rocker)
	* 5 -- Non-Grashof (pi-pi Rocker)
	* 6 -- Non-Grashof (pi-0 Rocker)
	* 7 -- Non-Grashof (0-pi Rocker)
	*/
	int getGrashofType(const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3) {
		double g = glm::length(p0 - p1);
		double a = glm::length(p0 - p2);
		double b = glm::length(p1 - p3);
		double h = glm::length(p2 - p3);

		double T1 = g + h - a - b;
		double T2 = b + g - a - h;
		double T3 = b + h - a - g;

		if (T1 < 0 && T2 < 0 && T3 >= 0) {
			return 0;
		}
		else if (T1 >= 0 && T2 >= 0 && T3 >= 0) {
			return 1;
		}
		else if (T1 >= 0 && T2 < 0 && T3 < 0) {
			return 2;
		}
		else if (T1 < 0 && T2 >= 0 && T3 < 0) {
			return 3;
		}
		else if (T1 < 0 && T2 < 0 && T3 < 0) {
			return 4;
		}
		else if (T1 < 0 && T2 >= 0 && T3 >= 0) {
			return 5;
		}
		else if (T1 >= 0 && T2 < 0 && T3 >= 0) {
			return 6;
		}
		else if (T1 >= 0 && T2 >= 0 && T3 < 0) {
			return 7;
		}
		else {
			return -1;
		}
	}

	/**
	* Check if the linkage has Grashof defect.
	* If the following conditions are not satisified, the linkage has Grashof defect, and true is returned.
	* - The sum of the shortest and longest link is less than the sum of the remaining links (i.e., s + l <= p + q).
	* - The shortest link is either a driving link or a ground link.
	* If both conditions are satisfied, there is no Grashof defect, and false is returned.
	*/
	bool checkGrashofDefect(const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3) {
		int linkage_type = getGrashofType(p0, p1, p2, p3);

		if (linkage_type == 0 || linkage_type == 1) {
			return false;
		}
		else {
			return true;
		}
	}

	/**
	* Check if the linkage has order defect.
	* If there is an order defect, true is returned.
	* Otherwise, false is returned.
	*/
	bool checkOrderDefect(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3) {
		int linkage_type = getGrashofType(p0, p1, p2, p3);

		glm::dvec2 inv_W = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(p2, 1));

		double total_cw = 0;
		double total_ccw = 0;
		double prev = 0;
		//int ccw = 1;
		for (int i = 0; i < poses.size(); i++) {
			// calculate the coordinates of the circle point of the driving crank in the world coordinate system
			glm::dvec2 X = glm::dvec2(poses[i] * glm::dvec3(inv_W, 1));
			//std::cout << X.x << "," << X.y << std::endl;

			// calculate the direction from the ground pivot (center point) of the driving crank to the circle point
			glm::dvec2 dir = X - p0;

			// calculate its angle
			double theta = atan2(dir.y, dir.x);

			if (i >= 1) {
				if (theta >= prev) {
					total_cw += kinematics::M_PI * 2 - theta + prev;
					total_ccw += theta - prev;
				}
				else {
					total_cw += prev - theta;
					total_ccw += kinematics::M_PI * 2 - prev + theta;
				}
			}

			prev = theta;
		}

		if (total_cw > kinematics::M_PI * 2 + 0.1 && total_ccw > kinematics::M_PI * 2 + 0.1) return true;
		else return false;
	}

	/**
	* Check if all the poses are in the same branch.
	* Drag-link and crank-rocker always do not have a branch defect.
	* For other types of linkage, the change in the sign of the angle between the coupler and the follower indicates the change of the branch.
	* If there is an branch defect, true is returned. Otherwise, false is returned.
	*
	* @param poses	pose matrices
	* @param p0		the world coordinates of the fixed point of the driving crank at the first pose
	* @param p1		the world coordinates of the fixed point of the follower at the first pose
	* @param p2		the world coordinates of the moving point of the driving crank at the first pose
	* @param p3		the world coordinates of the moving point of the follower at the first pose
	* @return		true if the branch defect is detected, false otherwise
	*/
	bool checkBranchDefectFor4RLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3) {
		int type = getGrashofType(p0, p1, p2, p3);

		// drag-link and crank-rocker always do not have a branch defect
		if (type == 0 || type == 1) return false;

		int orig_sign = 1;

		// calculate the local coordinates of the circle points
		glm::dvec2 W1 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(p2, 1));
		glm::dvec2 W2 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(p3, 1));

		for (int i = 0; i < poses.size(); i++) {
			// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
			glm::dvec2 X1 = glm::dvec2(poses[i] * glm::dvec3(W1, 1));
			glm::dvec2 X2 = glm::dvec2(poses[i] * glm::dvec3(W2, 1));

			// calculate its sign
			if (i == 0) {
				orig_sign = crossProduct(X2 - p1, X1 - X2) >= 0 ? 1 : -1;
			}
			else {
				int sign = crossProduct(X2 - p1, X1 - X2) >= 0 ? 1 : -1;
				if (sign != orig_sign) return true;
			}
		}

		return false;
	}

	bool checkCircuitDefectFor4RLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3) {
		int type = getGrashofType(p0, p1, p2, p3);

		// Non-grashof type does not have a circuit defect
		if (type >= 4) return false;

		int orig_sign0 = 1;
		int orig_sign1 = 1;
		int orig_sign2 = 1;
		int orig_sign3 = 1;

		// calculate the local coordinates of the circle points
		glm::dvec2 W1 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(p2, 1));
		glm::dvec2 W2 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec3(p3, 1));

		for (int i = 0; i < poses.size(); i++) {
			// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
			glm::dvec2 X1 = glm::dvec2(poses[i] * glm::dvec3(W1, 1));
			glm::dvec2 X2 = glm::dvec2(poses[i] * glm::dvec3(W2, 1));

			// calculate its sign
			if (i == 0) {
				orig_sign0 = crossProduct(p0 - p1, X1 - p0) >= 0 ? 1 : -1;
				orig_sign1 = crossProduct(p1 - X2, p0 - p1) >= 0 ? 1 : -1;
				orig_sign2 = crossProduct(X1 - p0, X2 - X1) >= 0 ? 1 : -1;
				orig_sign3 = crossProduct(X2 - X1, p1 - X2) >= 0 ? 1 : -1;
			}
			else {
				int sign0 = crossProduct(p0 - p1, X1 - p0) >= 0 ? 1 : -1;
				int sign1 = crossProduct(p1 - X2, p0 - p1) >= 0 ? 1 : -1;
				int sign2 = crossProduct(X1 - p0, X2 - X1) >= 0 ? 1 : -1;
				int sign3 = crossProduct(X2 - X1, p1 - X2) >= 0 ? 1 : -1;

				if (type == 0) {
					if (sign2 != orig_sign2 || sign3 != orig_sign3) return true;
				}
				else if (type == 1) {
					if (sign1 != orig_sign1 || sign3 != orig_sign3) return true;
				}
				else if (type == 2) {
					if (sign0 != orig_sign0 || sign2 != orig_sign2) return true;
				}
				else if (type == 3) {
					if (sign0 != orig_sign0 || sign1 != orig_sign1) return true;
				}
			}
		}

		return false;
	}

	bool checkCollisionFor4RLinkage(const std::vector<glm::dmat3x3>& poses, const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, const std::vector<std::vector<glm::dvec2>>& fixed_body_pts, const std::vector<glm::dvec2>& body_pts) {
		kinematics::Kinematics kinematics(0.02);

		// construct a linkage
		kinematics.diagram.addJoint(boost::shared_ptr<PinJoint>(new PinJoint(0, true, p0)));
		kinematics.diagram.addJoint(boost::shared_ptr<PinJoint>(new PinJoint(1, true, p1)));
		kinematics.diagram.addJoint(boost::shared_ptr<PinJoint>(new PinJoint(2, false, p2)));
		kinematics.diagram.addJoint(boost::shared_ptr<PinJoint>(new PinJoint(3, false, p3)));
		kinematics.diagram.addLink(true, kinematics.diagram.joints[0], kinematics.diagram.joints[2]);
		kinematics.diagram.addLink(false, kinematics.diagram.joints[1], kinematics.diagram.joints[3]);
		kinematics.diagram.addLink(false, kinematics.diagram.joints[2], kinematics.diagram.joints[3]);

		// set the geometry
		for (int i = 0; i < fixed_body_pts.size(); i++) {
			kinematics.diagram.addBody(kinematics.diagram.joints[0], kinematics.diagram.joints[1], fixed_body_pts[i]);
		}
		kinematics.diagram.addBody(kinematics.diagram.joints[2], kinematics.diagram.joints[3], body_pts);

		kinematics.diagram.initialize();

		// calculate the rotational angle of the driving crank for each pose
		std::vector<double> angles(poses.size());
		glm::dvec2 w(glm::inverse(poses[0]) * glm::dvec3(p2, 1));
		for (int i = 0; i < poses.size(); i++) {
			glm::dvec2 W = glm::dvec2(poses[i] * glm::dvec3(w, 1));
			angles[i] = atan2(W.y - p0.y, W.x - p0.x);
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
		int unvisited = poses.size() - 1;

		// run forward until collision is deteted or all the poses are reached
		while (true) {
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

			try {
				kinematics.stepForward(true, false);
			}
			catch (char* ex) {
				// if only some of the poses are reached before collision, the collision is detected.
				kinematics.clear();
				return true;
			}
		}

		kinematics.clear();
		return false;
	}

}