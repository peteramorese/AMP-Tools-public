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
    Vector2d vector = point2 - point1;
    double angle = std::atan2(vector.y(), vector.x());
    if (angle < 0) angle += 2 * M_PI;
    // cout << vector(0) << ", " << vector(1) << "\n";
    // cout << angle*180/M_PI << "\n\n";
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
    // cSpaceObstacle.pop_back();
    amp::Polygon poly(cSpaceObstacle);
    return  poly;
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
