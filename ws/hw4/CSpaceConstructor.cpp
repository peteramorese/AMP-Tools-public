#include "CSpaceConstructor.h"
#include "MyLinkManipulator.h"
#include <Eigen/Dense>
#include "HelpfulClass.h"

using namespace amp;
using std::vector, std::string, std::cout, Eigen::Vector2d;

// bool isPointInsidePolygon(const Vector2d& point, const vector<Vector2d>& polygon) {
//     int numVertices = polygon.size();
//     bool inside = false;
//     for (int i = 0, j = numVertices - 1; i < numVertices; j = i++) {
//         const Vector2d& vertex1 = polygon[i];
//         const Vector2d& vertex2 = polygon[j];
//         if ((vertex1.y() > point.y()) != (vertex2.y() > point.y()) &&
//             point.x() < (vertex2.x() - vertex1.x()) * (point.y() - vertex1.y()) / (vertex2.y() - vertex1.y()) + vertex1.x()) {
//             inside = !inside;
//         }
//     }
//     return inside;
// }

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
    Vector2d point = prevJoint;
    Vector2d line = currJoint - prevJoint;
    int steps = 50;
    Vector2d unitVector = line.normalized() * line.norm()/steps;
    for (int i=0; i<steps; ++i) {
        for (const amp::Obstacle2D& obstacle : allObstacles) {
            isInside = isPointInsidePolygon(point, obstacle.verticesCW());
            if (isInside) break;
        }
        point += unitVector;
        if (isInside) break;
    }
    return isInside;
};

bool CSpaceConstructor::inCollision(double x0, double x1) const {
    return false;
};

std::pair<std::size_t, std::size_t> CSpaceConstructor::getCellFromPoint(double x0, double x1) const {
    
};

