#include "CSpaceConstructor.h"
#include "MyLinkManipulator.h"
#include <Eigen/Dense>

using namespace amp;
using std::vector, std::string, std::cout, Eigen::Vector2d;

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

CSpaceConstructor::CSpaceConstructor(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
    : GridCSpace2D(x0_cells, x1_cells,  x0_min, x0_max, x1_min, x1_max) {}
            // : ConfigurationSpace2D(x0_min, x0_max, x1_min, x1_max)
            // , DenseArray2D<bool>(x0_cells, x1_cells)
            // {}


void CSpaceConstructor::populateGrid(const vector<double>& linkLengths, const vector<amp::Obstacle2D>& obstacles)  {
    MyLinkManipulator manipulator(linkLengths);
    // obstacles = obstacles;
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
                collision  = checkCollision(prevJoint, currJoint);
                if (collision) {
                    // cout << i << ", " << j << "\n";
                    break;
                };
            }
            if (collision) operator()(i, j) = 1;
            else operator()(i, j) = 0;
        }
    }
    // cout << operator()(1, 1) << "\n";
};

bool CSpaceConstructor::checkCollision(const Vector2d& prevJoint, const Vector2d& currJoint) {
	bool inside = false;
    Vector2d point = prevJoint;
    Vector2d line = currJoint - prevJoint;
    Vector2d unitVector = line.normalized();
    int steps = 50;
    double increment = line.norm()/steps;
    for (const amp::Obstacle2D& obstacle : allObstacles) {
        if (inside) break;
        for (int i=0; i<steps; ++i) {
            inside = isPointInsidePolygon(point, obstacle.verticesCW());
            point = point + unitVector * increment;
            if (inside) break;
        }
    }
    return inside;
};

bool CSpaceConstructor::inCollision(double x0, double x1) const {
    return false;
};

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
