#include "HelpfulClass.h"
#include <Eigen/Dense>

using std::vector, std::string, std::cout, Eigen::Vector2d, Eigen::VectorXd;

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

vector<vector<Edge>> findEdges(const vector<amp::Obstacle2D>& obstacles) {
	vector<vector<Edge>> edges;
	for (const amp::Obstacle2D& obstacle : obstacles) {
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

void smoothPath(amp::Path2D& path, const vector<amp::Obstacle2D> obstacles) {
    int size, i, j;
    for (int x = 0; x < 100; ++x) {
        size = path.waypoints.size();
        if (size < 3) break;
        i = amp::RNG::randi(0, size - 1);
        j = amp::RNG::randi(i + 1, size);
        if ((j - i) > 1 && !isLineInCollision(path.waypoints[i], path.waypoints[j], obstacles)) {
            path.waypoints.erase(path.waypoints.begin() + i + 1, path.waypoints.begin() + j);
        }
    }   
}

vector<vector<vector<Edge>>> findRegions(const vector<vector<Edge>>& allEdges) {
    vector<vector<vector<Edge>>> regions;
    for (const vector<Edge>& polyEdges : allEdges) {
        bool first = true;
        vector<vector<Edge>> polyRegions;
        Edge line1, line2, line3, firstLine;
        for (const Edge& edge : polyEdges) {
            Vector2d pointA = {edge.points.second.y() - edge.points.first.y(), edge.points.first.x() - edge.points.second.x()};
            Vector2d pointB = {edge.points.first.y() - edge.points.second.y(), edge.points.second.x() - edge.points.first.x()};
            line1 = findLineEquation(edge.points.first, pointA + edge.points.first);
            if (!first) {
                polyRegions.push_back({line3, line1});
            } else {
                firstLine = line1;
                first = false;
            }
            line2 = findLineEquation(edge.points.second, edge.points.first);
            line3 = findLineEquation(edge.points.second, pointB + edge.points.second);
            polyRegions.push_back({line1, line2, line3});
        }
        polyRegions.push_back({line3, firstLine});
        regions.push_back(polyRegions);
    }
    return regions;
}

double findClosestDistance(const Vector2d state, const vector<vector<Edge>>& polyRegions) {
    bool left;
    int ind = 0;
    for (const vector<Edge>& region : polyRegions) {
        bool allPass = true;
        if (region.size() == 2) left = false;
        else left = true;
        ind++;
        for (const Edge& edge : region) {
            if (!checkLine(state, edge, left)) {
                allPass = false; 
                break;
            }
        }
        if (allPass) {
            Vector2d closestPoint;
            if (region.size() == 2) closestPoint = region[0].points.first;
            else closestPoint = closestPointOnLine(state, region[1]);
            return (state - closestPoint).norm();
        }
    }
    cout << "\nCRASHED\n";
    return 0;
}

bool checkRobotOverlap(const VectorXd state, const vector<double> radii) {
    int m = radii.size();
    double norm;
    Vector2d center1, center2;
    for (int i = 0; i < m; i++) {
        for (int j = i + 1; j < m; j++) { 
            center1 = {state(2*i), state(2*i + 1)};
            center2 = {state(2*j), state(2*j + 1)};
            norm = (center1 - center2).norm();
            if (norm < (radii[i] + radii[j])) return true;
        }
    }
    return false;
}

vector<polygon> ampToBoostObstacles(const vector<amp::Obstacle2D>& obstacles) {
	vector<polygon> polygons;
	for (const amp::Obstacle2D& obstacle : obstacles) {
		std::string points = "POLYGON((";
		vector<Vector2d> vertices = obstacle.verticesCCW();
		vertices.push_back(vertices[0]);
		for (int j = 0; j < vertices.size(); ++j) {
			points += std::to_string(vertices[j](0)) + " " + std::to_string(vertices[j](1));
            if (j == vertices.size() - 1) points += "))";
            else points += ",";
		}
        cout << points << "\n"; // COUTTTT
        polygon poly;
        boost::geometry::read_wkt(points, poly);
		polygons.push_back(poly);
	}
	return polygons;
}

vector<polygon> vecToBoostObstacles(const vector<vector<Vector2d>>& obstacles) {
	vector<polygon> polygons;
	for (const std::vector<Eigen::Vector2d>& obstacle : obstacles) {
		std::string points = "POLYGON((";
        vector<Vector2d> vertices = obstacle;
		vertices.push_back(vertices[0]);
		for (int j = 0; j < vertices.size(); ++j) {
			points += std::to_string(vertices[j](0)) + " " + std::to_string(vertices[j](1));
            if (j == vertices.size() - 1) points += "))";
            else points += ",";
		}
        cout << points << "\n"; // COUTTTT
        polygon poly;
        boost::geometry::read_wkt(points, poly);
		polygons.push_back(poly);
	}
	return polygons;
}

std::vector<double> convertEigenToStd(const Eigen::VectorXd& eigenVec) {
    std::vector<double> stdVec(eigenVec.size());
    for (int i = 0; i < eigenVec.size(); ++i) {
        stdVec[i] = eigenVec(i);
    }
    return stdVec;
}

Eigen::VectorXd convertStdToEigen(const std::vector<double>& stdVec) {
    Eigen::VectorXd eigenVec(stdVec.size());
    for (int i = 0; i < stdVec.size(); ++i) {
        eigenVec(i) = stdVec[i];
    }
    return eigenVec;
}

vector<std::pair<double, double>> getRectangleVertices(const std::vector<double>& state, double w, double l) {
    double cx = state[0];
    double cy = state[1];
    double theta = state[2];
    const double TR_x = cx + ((w / 2) * cos(theta)) - ((l / 2) * sin(theta));
    const double TR_y = cy + ((w / 2) * sin(theta)) + ((l / 2) * cos(theta));
    std::string top_right = std::to_string(TR_x) + " " + std::to_string(TR_y);
    // TOP LEFT VERTEX:
    const double TL_x = cx - ((w / 2) * cos(theta)) - ((l / 2) * sin(theta));
    const double TL_y = cy - ((w / 2) * sin(theta)) + ((l / 2) * cos(theta));
    std::string top_left = std::to_string(TL_x) + " " + std::to_string(TL_y);
    // BOTTOM LEFT VERTEX:
    const double BL_x = cx - ((w / 2) * cos(theta)) + ((l / 2) * sin(theta));
    const double BL_y = cy - ((w / 2) * sin(theta)) - ((l / 2) * cos(theta));
    std::string bottom_left = std::to_string(BL_x) + " " + std::to_string(BL_y);
    // BOTTOM RIGHT VERTEX:
    const double BR_x = cx + ((w / 2) * cos(theta)) + ((l / 2) * sin(theta));
    const double BR_y = cy + ((w / 2) * sin(theta)) - ((l / 2) * cos(theta));
    return {{BR_x, BR_y}, {BL_x, BL_y}, {TL_x, TL_y}, {TR_x, TR_y}, {BR_x, BR_y}};
}

Eigen::VectorXd sampleFromRegion(const vector<Eigen::VectorXd>& polytope) {
    // Find bounding box
    int dim = polytope[0].size();
    if (dim == 3) { // Assuming the first vertex determines the dimensionality
        // Find bounding box
        vector<Eigen::Vector3d> poly;
        for (auto vertex : polytope) 
            poly.push_back({vertex(0) , vertex(1), vertex(2)});

        double minX = std::numeric_limits<double>::max();
        double minY = std::numeric_limits<double>::max();
        double minZ = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::lowest();
        double maxY = std::numeric_limits<double>::lowest();
        double maxZ = std::numeric_limits<double>::lowest();
        for (const auto& vertex : polytope) {
            minX = std::min(minX, vertex(0));
            maxX = std::max(maxX, vertex(0));
            minY = std::min(minY, vertex(1));
            maxY = std::max(maxY, vertex(1));
            minZ = std::min(minZ, vertex(2));
            maxZ = std::max(maxZ, vertex(2));
        }

        // Random number generator
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> distX(minX, maxX);
        std::uniform_real_distribution<double> distY(minY, maxY);
        std::uniform_real_distribution<double> distZ(minZ, maxZ);

        // Sample random points until a point inside the tetrahedron is found
        Eigen::Vector3d randomPoint(dim);
        do {
            randomPoint(0) = distX(gen);
            randomPoint(1) = distY(gen);
            randomPoint(2) = distZ(gen);
        } while (!isInsideTetrahedron(poly, randomPoint));

        return randomPoint;
    } else {
        vector<Eigen::Vector2d> poly;
        for (auto vertex : polytope) 
            poly.push_back({vertex(0) , vertex(1)});
        

        double minX = std::numeric_limits<double>::max();
        double minY = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::lowest();
        double maxY = std::numeric_limits<double>::lowest();
        for (const auto& vertex : polytope) {
            minX = std::min(minX, vertex.x());
            minY = std::min(minY, vertex.y());
            maxX = std::max(maxX, vertex.x());
            maxY = std::max(maxY, vertex.y());
        }

        // Random number generator
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> distX(minX, maxX);
        std::uniform_real_distribution<double> distY(minY, maxY);

        // Sample random points until a point inside the polygon is found
        Eigen::VectorXd randomPoint(dim);
        do {
            randomPoint(0) = distX(gen);
            randomPoint(1) = distY(gen);
        } while (!isPointInsidePolygon(randomPoint, poly));

        return randomPoint;
    }
}

double triangleArea(const std::array<Eigen::Vector2d, 3>& vertices) {
    Eigen::Vector2d v1 = vertices[0];
    Eigen::Vector2d v2 = vertices[1];
    Eigen::Vector2d v3 = vertices[2];

    double a = (v1 - v2).norm();
    double b = (v2 - v3).norm();
    double c = (v3 - v1).norm();
    double s = (a + b + c) / 2.0; // semiperimeter
    return sqrt(s * (s - a) * (s - b) * (s - c));
}

bool SameSide(Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d v3, Eigen::Vector3d v4, Eigen::Vector3d p) {
    VectorXd normal = (v2 - v1).cross(v3 - v1);
    double dotV4 = normal.dot(v4 - v1);
    double dotP = normal.dot(p - v1);
    return (dotV4 * dotP) >= 0.0;
}

bool isInsideTetrahedron(const vector<Eigen::Vector3d>& vertices, const Eigen::Vector3d& p) {
    Eigen::Vector3d v1 = vertices[0];
    Eigen::Vector3d v2 = vertices[1];
    Eigen::Vector3d v3 = vertices[2];
    Eigen::Vector3d v4 = vertices[3];

    return SameSide(v1, v2, v3, v4, p) &&
           SameSide(v2, v3, v4, v1, p) &&
           SameSide(v3, v4, v1, v2, p) &&
           SameSide(v4, v1, v2, v3, p);
}

bool isPointInsideRegion(const VectorXd& point, const vector<VectorXd>& polytope) {
    if (point.size() == 3) {
        vector<Eigen::Vector3d> poly;
        for (auto vertex : polytope) 
            poly.push_back({vertex(0) , vertex(1), vertex(2)});
        return isInsideTetrahedron(poly, point);
    } else if (point.size() == 3) {
        vector<Eigen::Vector2d> poly;
        for (auto vertex : polytope) 
            poly.push_back({vertex[0] , vertex[1]});
        return isPointInsidePolygon(point, poly);
    }
    return false;
}

double tetrahedronVolume(const std::array<Eigen::Vector3d, 4>& vertices) {
    // Extract vertices
    const Eigen::Vector3d& A = vertices[0];
    const Eigen::Vector3d& B = vertices[1];
    const Eigen::Vector3d& C = vertices[2];
    const Eigen::Vector3d& D = vertices[3];

    // Calculate the vectors
    Eigen::Vector3d BA = B - A;
    Eigen::Vector3d CA = C - A;
    Eigen::Vector3d DA = D - A;

    // Calculate the scalar triple product
    double scalarTripleProduct = BA.dot(CA.cross(DA));

    // Calculate the absolute value and divide by 6 to get the volume
    return abs(scalarTripleProduct) / 6.0;
}

Eigen::Vector3d tetrahedronCentroid(const std::array<Eigen::Vector3d, 4>& vertices) {
    // Calculate the centroid as the average of the vertices
    Eigen::Vector3d centroid(0, 0, 0);
    for (const auto& vertex : vertices) {
        centroid += vertex;
    }
    centroid /= 4.0;

    return centroid;
}

bool isPointInsideCube(const Eigen::Vector3d& point, const vector<Eigen::Vector3d>& cubeVertices) {
    // Find the minimum and maximum coordinates along each dimension
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double minZ = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double maxY = std::numeric_limits<double>::lowest();
    double maxZ = std::numeric_limits<double>::lowest();

    for (const auto& vertex : cubeVertices) {
        minX = std::min(minX, vertex(0));
        minY = std::min(minY, vertex(1));
        minZ = std::min(minZ, vertex(2));
        maxX = std::max(maxX, vertex(0));
        maxY = std::max(maxY, vertex(1));
        maxZ = std::max(maxZ, vertex(2));
    }

    // Check if the point lies within the range of the cube along each dimension
    return point(0) >= minX && point(0) <= maxX &&
           point(1) >= minY && point(1) <= maxY &&
           point(2) >= minZ && point(2) <= maxZ;
}