#include "CSpaceConstructor.h"
#include "MyLinkManipulator.h"
#include <Eigen/Dense>
#include "HelpfulClass.h"

using namespace amp;
using std::vector, std::string, std::cout, Eigen::Vector2d;

Edge findLineEquation(Vector2d point1, Vector2d point2) {
	double buffer = 0;
	double a, b, c;
	if (point1.x() == point2.x()) {
		a = 1;
		b = 0;
		c = point1.x();
	}
	else {
		a = -(point2.y() - point1.y()) / (point2.x() - point1.x());
		b = 1.0;
		c = a * point1.x() + point1.y();
	}
	vector<double> limitX = { point1.x(), point2.x() };
	vector<double> limitY = { point1.y(), point2.y() };
	sort(limitX.begin(), limitX.end());
	sort(limitY.begin(), limitY.end());

	Limits limits = {
		{limitX[0] - buffer, limitX[1] + buffer},
		{limitY[0] - buffer, limitY[1] + buffer} };
	Edge edge = {
		{a, b, c},
		limits,
		{point1, point2} };
	return edge;
}

bool findCollision(const Edge& edge,const Vector2d& point1, const Vector2d& point2) {
    bool condition = true;
    double x = point2.x();
    double y = point2.y();
    if (true) {
        if (edge.coeff.a == 0) {
            condition = edge.limits.x[0] < x && x < edge.limits.x[1];
        } else if (edge.coeff.b == 0) {
            condition = edge.limits.y[0] < y && y < edge.limits.y[1];
        } else {
            condition = edge.limits.x[0] < x && x < edge.limits.x[1] && edge.limits.y[0] < y && y < edge.limits.y[1];
        }
    }
    if (condition) {
        bool previousSide = edge.coeff.a * point1.x() + edge.coeff.b * point1.y() < edge.coeff.c;
        bool currentSide = edge.coeff.a * point2.x() + edge.coeff.b * point2.y() < edge.coeff.c;
        return (previousSide != currentSide);
    }
    else {
        return false;
    }
}

bool isPointInsidePolygon(const Vector2d& point, const vector<Vector2d>& polygon) {
    int numVertices = polygon.size();
    bool inside = false;
    for (int i = 0, j = numVertices - 1; i < numVertices; j = i++) {
        const Vector2d& vertex1 = polygon[i];
        const Vector2d& vertex2 = polygon[j];
        if ((vertex1.y() > point.y()) != (vertex2.y() > point.y()) &&
            point.x() < (vertex2.x() - vertex1.x()) * (point.y() - vertex1.y()) / (vertex2.y() - vertex1.y()) + vertex1.x()) {
            inside = !inside;
        }
    }
    return inside;
}

vector<Edge> findEdges(const vector<amp::Obstacle2D>& obstacles) {
	vector<Edge> edges;
	for (const amp::Obstacle2D& obstacle : obstacles) {
		vector<Eigen::Vector2d> vertices = obstacle.verticesCW();
		vertices.push_back(vertices[0]);
		for (int j = 1; j < vertices.size(); ++j) {
			Edge edge = findLineEquation({ vertices[j - 1](0), vertices[j - 1](1) }, { vertices[j](0), vertices[j](1) });
			edge.edgeInd = j - 1;
			edges.push_back(edge);
		}
	}
	return edges;
}

// std::unique_ptr<amp::GridCSpace2D> GradingConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
//     std::unique_ptr<amp::GridCSpace2D> p;
//     return p;
// };


CSpaceConstructor::CSpaceConstructor(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
    : GridCSpace2D(x0_cells, x1_cells,  x0_min, x0_max, x1_min, x1_max) {}
            // : ConfigurationSpace2D(x0_min, x0_max, x1_min, x1_max)
            // , DenseArray2D<bool>(x0_cells, x1_cells)
            // {}


void CSpaceConstructor::populateGrid(const vector<double>& linkLengths, const vector<amp::Obstacle2D>& obstacles)  {
    MyLinkManipulator manipulator(linkLengths);
    allObstacles = obstacles;
	edges = findEdges(obstacles);
    std::pair<int, int> gridSize = size();
    int x0 = gridSize.first;
    int x1 = gridSize.second;
    double t0;
    double t1;
    Vector2d prevJoint;
    Vector2d currJoint;
    bool collision;
    for (int i = 0; i < x0; ++i){
        t0 = 2*i*M_PI / x0;
        for (int j = 0; j < x1; ++j){
            collision = false;
            t1 = 2*j*M_PI / x1;
            for (int k=0; k < linkLengths.size(); ++k) {
                prevJoint = manipulator.getJointLocation({t0, t1}, k);
                currJoint = manipulator.getJointLocation({t0, t1}, k+1);
                // cout << "Point: (" << prevJoint.x() << ", " << prevJoint.y() << ")\n";
                // cout << "Point: (" << currJoint.x() << ", " << currJoint.y() << ")\n\n";
                collision  = checkCollision(prevJoint, currJoint);
                if (collision) {
                    // cout << i << ", " << j << "\n";
                    break;
                };
            }
            if (collision) {
                operator()(i, j) = 1;
            };
        }
    }
    // cout << operator()(1, 1) << "\n";
};

bool CSpaceConstructor::checkCollision(const Vector2d& prevJoint, const Vector2d& currJoint) {
	bool isInside = false;
    Vector2d point;
    Vector2d prevPoint = prevJoint;
    Vector2d line = currJoint - prevJoint;
    int steps = 50;
    Vector2d unitVector = line.normalized() * line.norm()/steps;
    // cout << "Line: (" << line.x() << ", " << line.y() << ")\n";
    // cout << "Unit Vector: (" << unitVector.x() << ", " << unitVector.y() << ")\n";
    // cout << "Point: (" << prevJoint.x() << ", " << prevJoint.y() << ")\n";
    // cout << "Point: (" << currJoint.x() << ", " << currJoint.y() << ")\n\n";
    for (int i=0; i<steps; ++i) {
        for (const amp::Obstacle2D& obstacle : allObstacles) {
            isInside = isPointInsidePolygon(point, obstacle.verticesCW());
            if (isInside) break;
        }
        point = prevPoint + unitVector;
        // for (const Edge& edge : edges) {
        //     isInside = findCollision(edge, prevPoint, point);
        //     if (isInside) break;
        // }
        // cout << "Point: (" << point.x() << ", " << point.y() << ")\n";
        // cout << "Point: (" << prevPoint.x() << ", " << prevPoint.y() << ")\n\n";
        prevPoint = point;
        if (isInside) break;
    }
    return isInside;
};

bool CSpaceConstructor::inCollision(double x0, double x1) const {
    if (x0 == 0 && x1 == 0) {
        // init();
    }
    return false;
};

void CSpaceConstructor::init() {

}
// ManipulatorState MyLinkManipulator::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
//     ManipulatorState state;
//     std::vector<double> links = getLinkLengths();
//     for (int i = 0; i < 12; ++i) {
//         double theta3 = 2 * M_PI / 12 * i;
//         double x = links[2]*cos(theta3) + end_effector_location(0);
//         double y = links[2]*sin(theta3) + end_effector_location(1);
        // cout << "Point: (" << x << ", " << y << ")\n";
//         double cosTheta2 = ((pow(x, 2) + pow(y, 2)) - (pow(links[0], 2) + pow(links[1], 2)))/(2*links[0]*links[1]);
//         double cosTheta1 = (x*(links[0] + links[1]*cosTheta2)+y*links[1]*sqrt(1-pow(cosTheta2, 2)))/(pow(x, 2) + pow(y, 2));
//         state = {acos(cosTheta1), acos(cosTheta2), theta3 - acos(cosTheta2) - acos(cosTheta1) - M_PI};
//         bool pass = true;
//         for (auto element : state) {            
//             cout << element << " ";
//             if (std::isnan(element)) pass = false;
//         }
//         cout << "\n";
//         if (pass) break;
//     }
//     return state;
// };
