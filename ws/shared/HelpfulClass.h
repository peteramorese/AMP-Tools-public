#pragma once
#include "AMPCore.h"
using std::vector, std::string, std::cout, Eigen::Vector2d;

class MyClass {
    public:
        amp::Polygon findMinkowskiDiff(const amp::Obstacle2D& obstacle, std::vector<Eigen::Vector2d> robotVertices);
        std::vector<amp::Polygon> findCSpaceObstacles(const amp::Obstacle2D& obstacle, std::vector<Eigen::Vector2d> robotVertices);

};

struct Coefficients
{
	double a;
	double b;
	double c;
};

struct Limits
{
	vector<double> x;
	vector<double> y;
};

struct Edge
{
	Coefficients coeff;
	Limits limits;
	vector<Vector2d> points;
	int edgeInd;
};
