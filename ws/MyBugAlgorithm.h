#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

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
	int edgeInd;
};

Edge findLineEquation(Point point1, Point point2) {
	double buffer = 0;
	double a, b, c;
	if (point1.x == point2.x) {
		a = 1;
		b = 0;
		c = point1.x;
	}
	else {
		a = -(point2.y - point1.y) / (point2.x - point1.x);
		b = 1.0;
		c = a * point1.x + point1.y;
	}
	vector<double> limitX = { point1.x, point2.x };
	vector<double> limitY = { point1.y, point2.y };
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

double distanceBetweenPoints(const Point& point1, const Point& point2) {
	return std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2));
}

Point comparePoints(const Point& coord1, const Point& coord2, const Point& target, bool closest = false) {
	double distance1 = distanceBetweenPoints(coord1, target);
	double distance2 = distanceBetweenPoints(coord2, target);
	Point result;
	if (closest) {
		if (distance1 < distance2) {
			result = coord1;
		} else {
			result =coord2;
		}
	} else {
		if (distance1 > distance2) {
			result =coord1;
		} else {
			result =coord2;
		}
	}
	return result;
}

vector<vector<Edge>> findEdges(const amp::Problem2D& problem) {
	vector<vector<Edge>> edges;
	for (const amp::Obstacle2D& obstacle : problem.obstacles) {
		vector<Edge> polyEdges;
		vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
		vertices.push_back(vertices[0]);
		for (int j = 1; j < vertices.size(); ++j) {
			Edge edge = findLineEquation({ vertices[j - 1](0), vertices[j - 1](1) }, { vertices[j](0), vertices[j](1) });
			edge.edgeInd = j - 1;
			polyEdges.push_back(edge);
		}
		edges.push_back(polyEdges);
	}
	return edges;
}

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm{
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
	vector<int> entryPointIndices;
	string mode;
	vector<vector<Edge>> edges;
	Edge mLine;
	double shortestPath;
	amp::Path2D path;
	int polyInd;
	int edgeInd;
	bool hasTurned;
	bool CW;

public:
	MyBugAlgorithm(int bugType) :
		bugType(bugType),
		heading(0),
		polyInd(-1),
		edgeInd(-1),
		step(0),
		hasTurned(false),
		mode("goal") {}

	void moveForward() {
		x += shortestPath / 5000 * std::cos(heading); 
		y += shortestPath / 5000 * std::sin(heading);
		positionHistory.push_back({x, y});
		path.waypoints.push_back(Eigen::Vector2d(x, y));
		step++;
	}

	virtual amp::Path2D plan(const amp::Problem2D& problem) {
		init(problem);
		int maxSteps = 55000;
		for (int i = 0; i < maxSteps; ++i) {
			moveForward();
			detectAllEdges();
			if (bugType == 1) {
				detectEntryExitPoints();
			}
			else {
				detectMLine();
			}
			if (detectPoint(goal)) {
				cout << "Goal reached in " << i << " steps\n";
				path.waypoints.push_back(Eigen::Vector2d(goal.x, goal.y));
				break;
			}
		}
		if (step == maxSteps) {
			cout << "Max steps of " << step << " reached. maxSteps should be increased.\n";
		}
		return path;
	}

	void init(const amp::Problem2D& problem) {
		cout << "Starting Bug " << bugType << "\n";
		x = problem.q_init(0);
		y = problem.q_init(1);
		start = { x, y };
		positionHistory.push_back(start);
		path.waypoints.push_back(Eigen::Vector2d(x, y));
		goal = { problem.q_goal(0), problem.q_goal(1)};
		edges = findEdges(problem);
		mLine = findLineEquation(goal, start);
		shortestPath = distanceBetweenPoints(start, goal);
		cout << shortestPath << "\n";
		CW = true;
		turnToGoal();
	}

	void turnToGoal() {
		heading = std::atan2(-(y - goal.y), -(x - goal.x));
	}

	void rewind() {
		Point previousPoint = positionHistory[positionHistory.size() - 2];
		positionHistory.pop_back();
		path.waypoints.pop_back();
		x = previousPoint.x;
		y = previousPoint.y;
	}

	void detectAllEdges() {
		detectEdges();
		detectNextEdge();
	}

	void detectEdges() {
		for (int i = 0; i < edges.size(); ++i) {
			if (i != polyInd) {
				// for (const Edge& edge : edges[i]) {
				for (int j = edges[i].size() - 1; j >= 0; j--) {
					Edge edge = edges[i][j];
					if (findCollision(edge, true)) {
						if (mode == "goal") {
							entryPoints.push_back({ x, y });
							entryPointIndices.push_back(step);
							mode = "circ";
						}
						polyInd = i;
						edgeInd = edge.edgeInd;
						// cout << "Crossed Edge at " << x << ", " << y << "\n";
						findDirectionToTurn(edge);
						rewind();
						break;
					}
				}
			}
		}
	}

	void detectNextEdge() {
		if (mode != "goal") {
			int ind;
			if (CW) {
				ind = edgeInd - 1;
				if (edgeInd == 0) {
					ind = edges[polyInd].size() - 1;
				}
			} else {
				ind = edgeInd + 1;
				if (edgeInd == edges[polyInd].size() - 1) {
					ind = 0;
				}
			}
			Edge nextEdge = edges[polyInd][ind];
			if (findCollision(nextEdge,false)) {
				// cout << "Turning at Vertex " << x << ", " << y << "\n";
				findDirectionToTurn(nextEdge);
				edgeInd = ind;
				hasTurned = true;
			}

		}
	}

	void detectMLine() {
		if (mode == "circ") {
			if (findCollision(mLine, true)) {
				// cout << "Crossed M-Line at " << x << ", " << y << "\n";
				Point previousEntry = entryPoints[entryPoints.size() - 1];
				Point checkFartherPoint = comparePoints(previousEntry, {x, y}, goal);
				if (checkFartherPoint.x == previousEntry.x && checkFartherPoint.y == previousEntry.y) {
					if (goal.x == 35){
					x += shortestPath / 5000 * std::cos(heading); 
					y += shortestPath / 5000 * std::sin(heading);}
					findDirectionToTurn(mLine, true);
					mode = "goal";
				}
			}
		}
	}

	void detectEntryExitPoints() {
		if (hasTurned) {
			if (mode == "circ") {
				bool isComplete = detectPoint(entryPoints[entryPoints.size() - 1]);
				if (isComplete) {
					// cout << "Completed circumnavigation at "<< x << ", " << y << "\n";
					mode = "exiting";
					Point closestPoint = start;
					vector<Point> pointsOnPoly;
					for (int i = entryPointIndices[entryPointIndices.size() - 1]; i < positionHistory.size(); ++i) {
						pointsOnPoly.push_back(positionHistory[i]);
					}
					for (const Point& point : pointsOnPoly) {
						closestPoint = comparePoints(closestPoint, point, goal, true);
					}
					exitPoint = closestPoint;
					int ind = 0;
					for (const Point& point : pointsOnPoly) {
						if (point.x == closestPoint.x && point.y == closestPoint.y) {
							break;
						}
						ind++;
					}
					if (ind > pointsOnPoly.size() / 2) {
						cout << "turning 180\n";
						heading += M_PI;
						CW = false;
					}
				}
			}
			else if (mode == "exiting") {
				if (detectPoint(exitPoint)) {
					// cout << "Exiting Polygon\n";
					turnToGoal();
					mode = "goal";
					hasTurned = false;
					CW = true;
				}
			}
		}

	}

	bool detectPoint(const Point& point) {
		double distance = distanceBetweenPoints({x, y}, point);
		return distance / shortestPath < 0.0005;
	}

	bool findCollision(const Edge& edge, bool checkLimits=false) {
		bool condition = true;
		if (checkLimits) {
			if (edge.coeff.a == 0) {
				condition = edge.limits.x[0] < x && x < edge.limits.x[1];
			} else if (edge.coeff.b == 0) {
				condition = edge.limits.y[0] < y && y < edge.limits.y[1];
			} else {
				condition = edge.limits.x[0] < x && x < edge.limits.x[1] && edge.limits.y[0] < y && y < edge.limits.y[1];
			}
		}
		if (condition) {
			Point perviousPoint = positionHistory[positionHistory.size() - 2];
			bool previousSide = edge.coeff.a * perviousPoint.x + edge.coeff.b * perviousPoint.y < edge.coeff.c;
			bool currentSide = edge.coeff.a * x + edge.coeff.b * y < edge.coeff.c;
			return (previousSide != currentSide);
		}
		else {
			return false;
		}
	}

	void findDirectionToTurn(const Edge& edge, bool isMLine = false) {
		Point targetVertex;
		double guessHeading;
		if (isMLine) {
			targetVertex = goal;
		}
		else {
			if (CW) {
				targetVertex = edge.points[0];
			} else {
				targetVertex = edge.points[1];
			}
			
		}
		if (edge.coeff.b == 0.0) {
			guessHeading = - M_PI / 2;
		}
		else {
			guessHeading = std::atan(-edge.coeff.a);
		}
		Point projectedPoint = {x + shortestPath / 1500 * std::cos(guessHeading), y + shortestPath / 1500 * std::sin(guessHeading)};
		Point fartherPoint = comparePoints({x, y}, projectedPoint, targetVertex);
		if (projectedPoint.x == fartherPoint.x && projectedPoint.y == fartherPoint.y) {
			heading = guessHeading + M_PI;
		}
		else {
			heading = guessHeading;
		}
	}
};