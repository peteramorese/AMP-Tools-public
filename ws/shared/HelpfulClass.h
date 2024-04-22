#pragma once
#include "AMPCore.h"
#include <boost/geometry/io/io.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>

using std::vector, std::cout, Eigen::Vector2d;
typedef boost::geometry::model::d2::point_xy<double> point;
typedef boost::geometry::model::polygon<point> polygon; 

struct Coefficients {
	double a;
	double b;
	double c;
};

struct Points {
	Vector2d first;
	Vector2d second;
};

struct Edge {
	Coefficients coeff;
	Points points;
};

class MyClass {
    public:
        amp::Polygon findMinkowskiDiff(const amp::Obstacle2D& obstacle, std::vector<Eigen::Vector2d> robotVertices);
        std::vector<amp::Polygon> findCSpaceObstacles(const amp::Obstacle2D& obstacle, std::vector<Eigen::Vector2d> robotVertices);
};

bool isPointInsidePolygon(const Vector2d& point, const vector<Vector2d>& polygon);

bool isPointInCollision(const Vector2d& point, const vector<amp::Obstacle2D> obstacles);

double distanceBetweenPoints(const Vector2d& point1, const Vector2d& point2);

Edge findLineEquation(const Vector2d& point1,const Vector2d& point2);

vector<vector<Edge>> findEdges(const vector<amp::Obstacle2D>& obstacles);

bool checkLine(const Vector2d& point, const Edge& edge, bool left);

double distanceToLine(const Vector2d& point, const Edge& edge);

Vector2d closestPointOnLine(const Vector2d& point, const Edge& edge);

bool doesLineIntersectPolygon(const Vector2d& p1, const Vector2d& p2, const vector<Vector2d>& polygon);

bool isLineInCollision(const Vector2d& point1, const Vector2d& point2, const vector<amp::Obstacle2D> obstacles);

void smoothPath(amp::Path2D& path, const vector<amp::Obstacle2D> obstacles);

// HW 8 Functions

vector<vector<vector<Edge>>> findRegions(const vector<vector<Edge>>& allEdges);

double findClosestDistance(const Vector2d state, const vector<vector<Edge>>& polyRegions);

bool checkRobotOverlap(const Eigen::VectorXd state, const vector<double> radii);

vector<polygon> ampToBoostObstacles(const vector<amp::Obstacle2D>& obstacles);

std::vector<double> convertEigenToStd(const Eigen::VectorXd& eigenVec);

Eigen::VectorXd convertStdToEigen(const std::vector<double>& stdVec);

vector<std::pair<double, double>> getRectangleVertices(const std::vector<double>& state, double w, double l);

Vector2d sampleFromRegion(const vector<Vector2d>& polygon);

