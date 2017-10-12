#include "BodyGeometry.h"
#include "Joint.h"
#include "KinematicUtils.h"

namespace kinematics {

	/**
	 * Get the actual coordinates of the body geometry.
	 * Note that "points" store the original coordinates in the model coordinate system.
	 */
	std::vector<glm::dvec2> BodyGeometry::getActualPoints() {
		std::vector<glm::dvec2> actual_points;

		glm::dmat3x2 model = getLocalToWorldModel();

		for (int k = 0; k < points.size(); ++k) {
			actual_points.push_back(model * glm::dvec3(points[k], 1));
		}

		return actual_points;
	}

	void BodyGeometry::draw(QPainter& painter, const QPointF& origin, float scale) {
		painter.save();

		painter.setPen(QPen(QColor(0, 0, 0), 1));
		painter.setBrush(QBrush(QColor(0, 255, 0, 60)));
		std::vector<glm::dvec2> actual_points = getActualPoints();
		QPolygonF pts;
		for (int k = 0; k < actual_points.size(); ++k) {
			pts.push_back(QPointF(origin.x() + actual_points[k].x * scale, origin.y() - actual_points[k].y * scale));
		}
		painter.drawPolygon(pts);

		painter.restore();

	}

	glm::dmat3x2 BodyGeometry::getLocalToWorldModel() {
		glm::dvec2 dir = pivot2->pos - pivot1->pos;
		double angle = atan2(dir.y, dir.x);
		glm::dvec2 p1 = pivot1->pos;

		glm::dmat3x2 model;
		model[0][0] = cos(angle);
		model[1][0] = -sin(angle);
		model[2][0] = p1.x;
		model[0][1] = sin(angle);
		model[1][1] = cos(angle);
		model[2][1] = p1.y;
		return model;
	}

	glm::dmat3x2 BodyGeometry::getWorldToLocalModel() {
		glm::vec2 dir = pivot2->pos - pivot1->pos;
		double angle = -atan2(dir.y, dir.x);

		glm::dmat3x2 model;
		model[0][0] = cos(angle);
		model[1][0] = -sin(angle);
		model[2][0] = -pivot1->pos.x * cos(angle) + pivot1->pos.y * sin(angle);
		model[0][1] = sin(angle);
		model[1][1] = cos(angle);
		model[2][1] = -pivot1->pos.x * sin(angle) - pivot1->pos.y * cos(angle);
		return model;
	}

}