#include "CSpaceConstructor.h"
#include "MyLinkManipulator.h"
#include <Eigen/Dense>
#include "HelpfulClass.h"

using namespace amp;
using std::vector, std::string, std::cout, Eigen::Vector2d;

CSpaceConstructor::CSpaceConstructor(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
    : GridCSpace2D(x0_cells, x1_cells,  x0_min, x0_max, x1_min, x1_max) {}
            // : ConfigurationSpace2D(x0_min, x0_max, x1_min, x1_max)
            // , DenseArray2D<bool>(x0_cells, x1_cells)
            // {}

Vector2d CSpaceConstructor::getPointFromCell(const std::pair<int, int>& cell) {
    std::pair<double, double> x0lim = x0Bounds();
    std::pair<double, double> x1lim = x1Bounds();
    std::pair<int, int> gridSize = size();
    int cells0 = gridSize.first;
    int cells1 = gridSize.second;
    int i = cell.first;
    int j = cell.second;
    double x = x0lim.first + i * (x0lim.second - x0lim.first) / cells0;
    double y = x1lim.first + j * (x1lim.second - x1lim.first) / cells1;
    return {x, y};
}

void CSpaceConstructor::populateGrid(const vector<amp::Obstacle2D>& obstacles)  {
    cout << "Finding Obstacles\n";
    std::pair<double, double> x0lim = x0Bounds();
    std::pair<double, double> x1lim = x1Bounds();    
    std::pair<int, int> gridSize = size();
    int cells0 = gridSize.first;
    int cells1 = gridSize.second;
    Vector2d point;
    for (int i = 0; i < cells0; ++i) {
        for (int j = 0; j < cells1; ++j) {
            point = getPointFromCell({i, j});
            bool collision = false;
            for (const amp::Obstacle2D& obstacle : obstacles) {
                collision = isPointInsidePolygon(point, obstacle.verticesCW());
                if (collision) operator()(i, j) = 1;
            }
        }
    }
}


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
                collision  = checkCollision(prevJoint, currJoint);
                if (collision) {
                    break;
                };
            }
            if (collision) operator()(i, j) = 1;
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

std::pair<std::size_t, std::size_t> CSpaceConstructor::getCellFromPoint(double x0, double x1) const {
    std::pair<double, double> x0lim = x0Bounds();
    std::pair<double, double> x1lim = x1Bounds();
    std::pair<int, int> gridSize = size();
    int cells0 = gridSize.first;
    int cells1 = gridSize.second;
    double cellWidth0 = (x0lim.second - x0lim.first)/cells0;
    double cellWidth1 = (x1lim.second - x1lim.first)/cells1;
    int i = static_cast<int>(std::floor((x0 - x0lim.first) / cellWidth0));
    int j = static_cast<int>(std::floor((x1 - x1lim.first) / cellWidth1));
    return std::pair<std::size_t, std::size_t>(i, j);
};

