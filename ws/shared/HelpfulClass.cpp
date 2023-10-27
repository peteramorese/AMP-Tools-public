#include "HelpfulClass.h"
#include <Eigen/Dense>

using std::vector, std::string, std::cout, Eigen::Vector2d;

bool findLower(const Vector2d& a, const Vector2d& b) {
    if (a.y() != b.y()) {
        return a.y() < b.y();
    } else {
        return a.x() < b.x(); 
    }
}

int findLowerLeft(vector<Vector2d> vertices) {
    int k = 0;
    for (int i = 1; i < vertices.size(); ++i) {
        if (findLower(vertices[i], vertices[k])) {
            k = i;
        }
    }
    return k;
}
 
vector<Vector2d> rearrangeVector(vector<Vector2d>& vertices,int startingIndex) {
    vector<Vector2d> rearrangedVector;
    for (size_t i = startingIndex; i < vertices.size(); ++i) {
        rearrangedVector.push_back(vertices[i]);
    }
    // Copy elements from the original vector, starting from the beginning up to startingIndex
    for (size_t i = 0; i < startingIndex; ++i) {
        rearrangedVector.push_back(vertices[i]);
    }
    return rearrangedVector;
}


double findAngle(const Vector2d& point1, const Vector2d& point2) {
    Vector2d vector_ = point2 - point1;
    double angle = std::atan2(vector_(1), vector_(0));
    if (angle < 0) angle += 2 * M_PI;
    return angle;
}

vector<Vector2d> flipSortVertices(vector<Vector2d>& vertices) {
	for (Vector2d& vertex: vertices) {
        vertex = Vector2d(-vertex(0), -vertex(1));
    }
    int ind = findLowerLeft(vertices);
    vector<Vector2d> rararrangedVector = rearrangeVector(vertices, ind);
    return rararrangedVector;
    // std::sort(vertices.begin(), vertices.end(), customComparator);
}

vector<Vector2d> rotateRobot(vector<Vector2d> robotVertices, const double angle) {
    vector<Vector2d> rotatedVertices;
    Eigen::Rotation2Dd rotation(angle);
    for (Vector2d& vertex: robotVertices) {
        rotatedVertices.push_back(rotation * vertex);
        // cout << vertex(0) << ", " << vertex(1) << "\n";
    }
    return rotatedVertices;
}

amp::Polygon MyClass::findMinkowskiDiff(const amp::Obstacle2D& obstacle, vector<Vector2d> robotVertices) {
    vector<Vector2d> obstacleVertices = obstacle.verticesCCW();
    obstacleVertices.push_back(obstacleVertices[0]);
    vector<Vector2d> formattedVertices = flipSortVertices(robotVertices);
    // std::sort(robotVertices.begin(), robotVertices.end(), customComparator);
    // robotVertices = { Vector2d(0, 0) , Vector2d(1, 0), Vector2d(1, 2) };
	for (Vector2d& vertex: formattedVertices) {
        cout << vertex(0) << ", " << vertex(1) << "\n";
    }
    formattedVertices.push_back(formattedVertices[0]);
    vector<Vector2d> cSpaceObstacle;
    int n = obstacleVertices.size();
    int m = formattedVertices.size();
    int i = 0;
    int j = 0;
    while (i < n && j < m) {
        Vector2d vertex = obstacleVertices[i] + formattedVertices[j];
        cout << "Adding vertex (" << vertex(0) << ", " << vertex(1) << ")\n";
        cSpaceObstacle.push_back(vertex);
        if (findAngle(obstacleVertices[i], obstacleVertices[i + 1]) < findAngle(formattedVertices[j], formattedVertices[j + 1])) {
            i++;
        }
        else if (findAngle(obstacleVertices[i], obstacleVertices[i + 1]) > findAngle(formattedVertices[j], formattedVertices[j + 1])) {
            j++;
        }
        else {
            i++;
            j++;
        }
    }
    amp::Polygon poly(cSpaceObstacle);
    return poly;
}

vector<amp::Polygon> MyClass::findCSpaceObstacles(const amp::Obstacle2D& obstacle, vector<Vector2d> robotVertices) {
    vector<amp::Polygon> polygons;
    vector<double> angles;
    for (int i = 0; i < 12; ++i) {
        angles.push_back(2*M_PI / 12 * i);
    }
    for (const double& angle: angles) {
        cout << angle << "\n";
        vector<Vector2d> rotatedVertices = rotateRobot(robotVertices, angle);
        // flipVertices(rotatedVertices);
        amp::Polygon poly = findMinkowskiDiff(obstacle, rotatedVertices);
        polygons.push_back(poly);
    }
    return polygons;
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

bool isPointInCollision(const Vector2d& point, const vector<amp::Obstacle2D> obstacles){
    bool isInside;
    for (const amp::Obstacle2D& obstacle : obstacles) {
        isInside = isPointInsidePolygon(point, obstacle.verticesCCW());
        if (isInside) return true;
    }
    return false;
}

double distanceBetweenPoints(const Vector2d& point1, const Vector2d& point2) {
	return (point2 - point1).norm();
}

Edge findLineEquation(const Vector2d& point1,const Vector2d& point2) {
	double a, b, c;
	a = point2.y() - point1.y();
	b = point1.x() - point2.x();
	c = point2.x() * point1.y() - point2.y() * point1.x();
    // cout << "Line Equation: " << a << "x + " << b << "y + " << c << " < 0\n";
	Edge edge = {
        {a, b, c},
		{point1, point2}};
	return edge;
}

vector<vector<Edge>> findEdges(const amp::Problem2D& problem) {
	vector<vector<Edge>> edges;
	for (const amp::Obstacle2D& obstacle : problem.obstacles) {
		vector<Edge> polyEdges;
		vector<Vector2d> vertices = obstacle.verticesCCW();
		vertices.push_back(vertices[0]);
		for (int j = 1; j < vertices.size(); ++j) {
			Edge edge = findLineEquation(vertices[j - 1],vertices[j]);
			polyEdges.push_back(edge);
		}
		edges.push_back(polyEdges);
	}
	return edges;
}

bool checkLine(const Vector2d& point, const Edge& edge, bool left) {
    bool isLeft = edge.coeff.a * point.x() + edge.coeff.b * point.y() + edge.coeff.c < 0;
    if (!left) isLeft = !isLeft;
    return isLeft;
}

double distanceToLine(const Vector2d& point, const Edge& edge) {
    double distance = std::abs(edge.coeff.a * point.x() + edge.coeff.b * point.y() + edge.coeff.c) / std::sqrt(pow(edge.coeff.a, 2) + pow(edge.coeff.b, 2));
    return distance;
}

Vector2d closestPointOnLine(const Vector2d& point, const Edge& edge) {
    double denominator = pow(edge.coeff.a, 2) + pow(edge.coeff.b, 2);
    double xPos = (edge.coeff.b * (edge.coeff.b * point.x() - edge.coeff.a * point.y()) - edge.coeff.a * edge.coeff.c) / denominator;
    double yPos = (edge.coeff.a * (-edge.coeff.b * point.x() + edge.coeff.a * point.y()) - edge.coeff.b * edge.coeff.c) / denominator;
    return {xPos, yPos};
}

double cross(const Vector2d& a, const Vector2d& b) {
    return a.x() * b.y() - a.y() * b.x();
}

double orient(const Vector2d& a, const Vector2d& b, const Vector2d& c) {
    return cross(b-a, c-a);
}

bool properInter(const Vector2d& a, const Vector2d& b, const Vector2d& c, const Vector2d& d) {
    double oa = orient(c,d,a), 
        ob = orient(c,d,b),            
        oc = orient(a,b,c),            
        od = orient(a,b,d);      
    // Proper intersection exists iff opposite signs  
    return (oa*ob < 0 && oc*od < 0);
} 

bool doesLineIntersectPolygon(const Vector2d& a, const Vector2d& b, const vector<Vector2d>& polygon) {
    if (isPointInsidePolygon(a, polygon) || isPointInsidePolygon(b, polygon)) {
        return true;  // At least one of the line endpoints is inside the polygon.
    }
    int n = polygon.size();
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        const Vector2d& c = polygon[i];
        const Vector2d& d = polygon[j];
        if (properInter(a, b, c, d)) return true;
    }
    return false;
}

bool isLineInCollision(const Vector2d& point1, const Vector2d& point2, const vector<amp::Obstacle2D> obstacles){
    bool doesIntersect;
    for (const amp::Obstacle2D& obstacle : obstacles) {
        doesIntersect = doesLineIntersectPolygon(point1, point2, obstacle.verticesCCW());
        if (doesIntersect) return true;
    }
    return false;
}