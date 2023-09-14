#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include <cmath>

using std::vector, std::string, std::cout;

struct Point
{
	double x;
	double y;
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
	vector<Point> points;
};

Edge findLineEquation(Point point1, Point point2)
{
	double buffer = 0.1;
	double a, b, c;
	if (point1.x == point2.x)
	{
		// auto [a, b, c] = tuple{ 1, 0, point1.x };
		a = 1;
		b = 0;
		c = point1.x;
	}
	else
	{
		a = -(point2.y - point1.y) / (point2.x - point1.x);
		b = 1.0;
		c = a * point1.x + point1.y;
	}
	vector<double> limitX = {point1.x, point2.x};
	vector<double> limitY = {point1.y, point2.y};
	sort(limitX.begin(), limitX.end());
	sort(limitY.begin(), limitY.end());

	Limits limits = {
		{limitX[0] - buffer, limitX[1] + buffer},
		{limitY[0] - buffer, limitY[1] + buffer}};

	cout << a << "x + " << b << "y = " << c << "\n";
	cout << limits.x[0] << ", " << limits.x[1] << ", " << limits.y[0] << ", " << limits.y[1] << "\n\n";
	Edge edge = {
		{a, b, c},
		limits,
		{point1, point2}};

	return edge;
}

double distanceBetweenPoints(const Point &point1, const Point &point2)
{
	return std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2));
}

Point comparePoints(const Point &coord1, const Point &coord2, const Point &target, bool closest = false)
{
	double distance1 = distanceBetweenPoints(coord1, target);
	double distance2 = distanceBetweenPoints(coord2, target);

	if (closest)
	{
		if (distance1 < distance2)
		{
			return coord1;
		}
		else
		{
			return coord2;
		}
	}
	else
	{
		if (distance1 > distance2)
		{
			return coord1;
		}
		else
		{
			return coord2;
		}
	}
}

vector<Edge> findEdges(const amp::Problem2D &problem)
{
	vector<Point> vertices;
	vector<Edge> edges;
	for (const amp::Obstacle2D &obstacle : problem.obstacles)
	{
		vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
		vertices.push_back(vertices[0]);
		for (int i = 1; i < vertices.size(); ++i)
		{
			// cout << vertices[i -1](0) << " , " << vertices[i -1 ](1) << "\n";
			// cout << vertices[i](0) << " , " << vertices[i](1) << "\n\n";
			edges.push_back(findLineEquation({vertices[i](0), vertices[i](1)}, {vertices[i - 1](0), vertices[i - 1](1)}));
		}
	}

	return edges;
}

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm
{
private:
	// Add any member variables here...
	int step;
	int bugType;
	double x;
	double y;
	double heading;
	Point start;
	Point goal;
	Point exitPoint;
	vector<Point> positionHistory;
	vector<Point> entryPoints;
	vector<Point> mLineIntercepts;
	vector<int> entryPointIndices;
	string mode;
	vector<Edge> edges;
	Edge mLine;
	int lockout;
	int lockMax;
	double shortestPath;
	amp::Path2D path;

public:
	MyBugAlgorithm(int bugType) : // start(start),
								  // goal(goal),
								  // edges(edges),
								  bugType(bugType),
								  // x(start.x),
								  // y(start.y),
								  heading(0),
								  lockout(0),
								  lockMax(5),
								  // positionHistory({ start }),
								  mode("goal")
	{
	}

	void moveForward()
	{
		x += shortestPath / 1000 * std::cos(heading);
		y += shortestPath / 1000 * std::sin(heading);
		positionHistory.push_back({x, y});
		path.waypoints.push_back(Eigen::Vector2d(x, y));
		lockout--;
		step++;
	}

	virtual amp::Path2D plan(const amp::Problem2D &problem)
	{
		cout << "Starting Bug " << bugType << "\n";
		x = problem.q_init(0);
		y = problem.q_init(1);
		start = {x, y};
		positionHistory.push_back(start);
		goal = {problem.q_goal(0), problem.q_goal(1)};
		edges = findEdges(problem);
		mLine = findLineEquation(start, goal);
		shortestPath = distanceBetweenPoints(start, goal);
		turnToGoal();
		for (int i = 0; i < 5000; ++i)
		{
			moveForward();
			detectEdges();
			if (bugType == 1)
			{
				detectEntryExitPoints();
			}
			else
			{
				detectMLine();
			}
			if (detectPoint(goal))
			{
				cout << "Goal reached in " << i << " steps\n";
				break;
			}
		}
		return path;
	}

	void turnToGoal()
	{
		heading = std::atan((y - goal.y) / (x - goal.x));
	}

	void detectEdges()
	{
		if (lockout < 0)
		{
			for (const Edge &edge : edges)
			{
				if (findCollision(edge))
				{
					if (mode == "goal")
					{
						entryPoints.push_back({x, y});
						entryPointIndices.push_back(step);
						mode = "circ";
					}
					cout << "Crossed Edge at " << x << ", " << y << "\n";
					lockout = lockMax;
					turnAlongEdge(edge);
				}
			}
		}
	}

	void detectMLine()
	{
		if (mode == "circ")
		{
			if (lockout < 0)
			{
				if (findCollision(mLine))
				{
					cout << "Crossed M-Line\n";
					lockout = lockMax;
					mLineIntercepts.push_back({x, y});
					Point previousEntry = entryPoints[entryPoints.size() - 1];
					Point checkFartherPoint = comparePoints(previousEntry, {x, y}, goal);
					if (checkFartherPoint.x == previousEntry.x && checkFartherPoint.y == previousEntry.y)
					{
						findDirectionToTurn(mLine, true);
						mode = "goal";
					}
				}
			}
		}
	}

	void detectEntryExitPoints()
	{
		if (mode == "circ")
		{
			if (lockout < 0)
			{
				bool isComplete = detectPoint(entryPoints[entryPoints.size() - 1]);
				if (isComplete)
				{
					cout << "Completed circumnavigation\n";
					mode = "exiting";
					Point closestPoint = start;
					// vector<Point>::iterator startIter = positionHistory.begin() + entryPointIndices[entryPointIndices.size() - 1];
					// vector<Point>::iterator endIter = positionHistory.end();
					// vector<Point> pointsOnPoly(startIter, endIter);
					vector<Point> pointsOnPoly;
					pointsOnPoly = vector<Point>(positionHistory.begin() + entryPointIndices[entryPointIndices.size() - 1], positionHistory.end());
					for (const Point &point : pointsOnPoly)
					{
						closestPoint = comparePoints(closestPoint, point, goal, true);
					}
					exitPoint = closestPoint;
					// vector<Point>::iterator it = std::find(pointsOnPoly.begin(), pointsOnPoly.end(), closestPoint);
					// int index = std::distance(pointsOnPoly.begin(), it);
					// if (index > pointsOnPoly.size() / 2) {
					// heading += M_PI;
				}
			}
		}
		else if (mode == "exiting")
		{
			if (detectPoint(exitPoint))
			{
				cout << "Exiting Polygon\n";
				lockout = lockMax;
				turnToGoal();
				mode = "goal";
			}
		}
	}

	bool detectPoint(const Point &point)
	{
		double distance = distanceBetweenPoints({x, y}, point);
		return distance / shortestPath < 0.005;
	}

	bool findCollision(const Edge &edge)
	{
		if (edge.limits.x[0] < x && x < edge.limits.x[1] && edge.limits.y[0] < y &&  y < edge.limits.y[1])
		{
			Point perviousPoint = positionHistory[positionHistory.size() - 2];
			bool previousSide = edge.coeff.a * perviousPoint.x + edge.coeff.b * perviousPoint.y < edge.coeff.c;
			bool currentSide = edge.coeff.a * x + edge.coeff.b * y < edge.coeff.c;
			return (previousSide != currentSide);
		}
		else
		{
			return false;
		}
	}

	void turnAlongEdge(const Edge &edge)
	{
		if (mode == "circ" || mode == "exiting")
		{
			findDirectionToTurn(edge);
		}
		else
		{
			heading = std::atan(-edge.coeff.a);
		}
	}

	void findDirectionToTurn(const Edge &edge, bool isMLine = false)
	{
		Point fartherstVertex;
		double guessHeading;
		if (isMLine)
		{
			fartherstVertex = goal;
		}
		else
		{
			fartherstVertex = comparePoints(edge.points[0], edge.points[1], {x, y});
		}
		if (edge.coeff.b == 0)
		{
			guessHeading = M_PI / 2;
		}
		else
		{
			guessHeading = std::atan(-edge.coeff.a);
		}
		Point projectedPoint = {x + std::cos(guessHeading), y + std::sin(guessHeading)};
		Point fartherPoint = comparePoints({x, y}, projectedPoint, fartherstVertex);
		if (projectedPoint.x == fartherPoint.x && projectedPoint.y == fartherPoint.y)
		{
			heading = guessHeading + M_PI;
		}
		else
		{
			heading = guessHeading;
		}
	}

	// Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
	// virtual amp::Path2D plan(const amp::Problem2D& problem) override;

	// Add any other methods here...
};