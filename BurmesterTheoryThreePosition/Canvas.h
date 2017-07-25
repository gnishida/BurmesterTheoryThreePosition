#ifndef CANVAS_H
#define CANVAS_H

#include <QWidget>
#include <QKeyEvent>
#include <glm/glm.hpp>
#include <boost/shared_ptr.hpp>
#include <kinematics.h>
#include <QTimer>
#include "Shape.h"
#include "Layer.h"
#include "Operation.h"
#include "History.h"

class MainWindow;

namespace canvas {

	class Canvas : public QWidget {
		Q_OBJECT

	public:
		static enum { MODE_SELECT = 0, MODE_MOVE, MODE_ROTATION, MODE_RESIZE, MODE_RECTANGLE, MODE_CIRCLE, MODE_POLYGON, MODE_LINKAGE_REGION, MODE_LINKAGE_AVOIDANCE, MODE_KINEMATICS };
		static enum { LINKAGE_4R = 0, LINKAGE_RRRP };

	public:
		MainWindow* mainWin;
		bool ctrlPressed;
		bool shiftPressed;

		int mode;
		boost::shared_ptr<Operation> operation;
		boost::shared_ptr<canvas::Shape> current_shape;
		std::vector<Layer> layers;
		int layer_id;
		boost::shared_ptr<canvas::Shape> selected_shape;
		std::vector<boost::shared_ptr<canvas::Shape>> copied_shapes;
		History history;
		
		std::vector<kinematics::Kinematics> kinematics;
		QTimer* animation_timer;
		bool collision_check;
		QPointF prev_mouse_pt;
		QPointF origin;
		double scale;
		std::vector<std::vector<std::vector<std::pair<glm::dvec2, glm::dvec2>>>> solutions;	// solutions[0] for the driving crank, solutions[1] for the follower
		std::pair<int, int> selectedJoint;
		//std::vector<bool> is_fixed_bodies;
		std::vector<std::vector<glm::dvec2>> fixed_body_pts;
		std::vector<std::vector<glm::dvec2>> body_pts;
		std::vector<std::vector<glm::dvec2>> linkage_region_pts;
		std::vector<std::vector<glm::dmat3x3>> poses;
		int linkage_type;
		int linkage_subtype;
		bool orderDefect;
		bool branchDefect;
		bool circuitDefect;

	public:
		Canvas(MainWindow* mainWin);
		~Canvas();

		void clear();
		void selectAll();
		void unselectAll();
		void deleteSelectedShapes();
		void undo();
		void redo();
		void copySelectedShapes();
		void pasteCopiedShapes();
		void circularRepeat(int num_repeat);
		void setMode(int mode);
		void setLayer(int layer_id);
		void open(const QString& filename);
		void save(const QString& filename);
		void run();
		void runBackward();
		void stop();
		void speedUp();
		void speedDown();
		void invertSpeed();
		void stepForward();
		void stepBackward();
		void showAssemblies(bool flag);
		void showLinks(bool flag);
		void showBodies(bool flag);
		glm::dvec2 screenToWorldCoordinates(const glm::dvec2& p);
		glm::dvec2 screenToWorldCoordinates(double x, double y);
		glm::dvec2 worldToScreenCoordinates(const glm::dvec2& p);

		void calculateSolutions(int linkage_type);
		int findSolution(const std::vector<std::pair<glm::dvec2, glm::dvec2>>& solutions, const glm::dvec2& pt);
		void updateDefectFlag(const std::vector<glm::dmat3x3>& poses, const kinematics::Kinematics& kinematics);

	public slots:
		void animation_update();

	protected:
		void paintEvent(QPaintEvent* e);
		void mousePressEvent(QMouseEvent* e);
		void mouseMoveEvent(QMouseEvent* e);
		void mouseReleaseEvent(QMouseEvent* e);
		void mouseDoubleClickEvent(QMouseEvent* e);
		void wheelEvent(QWheelEvent* e);
		void resizeEvent(QResizeEvent *e);

	public:
		void keyPressEvent(QKeyEvent* e);
		void keyReleaseEvent(QKeyEvent* e);
	};

}

#endif // CANVAS_H
