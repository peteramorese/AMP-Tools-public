#include "MyBugAlgorithm.h"
#include <vector>
#include <cmath>
#include "tools/Obstacle.h" 
#include "Line.h"

Line::Line(Eigen::Vector2d p1, Eigen::Vector2d p2) : point1(p1), point2(p2) {
    m = (p2[1] - p1[1]) / (p2[0] - p1[0]);
};

Line::~Line() {};
