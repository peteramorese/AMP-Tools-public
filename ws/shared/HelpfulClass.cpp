#include "HelpfulClass.h"

using std::vector, std::string, std::cout, Eigen::Vector2d;

double findAngle(const Vector2d& point1, const Vector2d& point2) {
    Vector2d vector = point2 - point1;
    double angle = std::atan2(vector.y(), vector.x());
    // cout << vector(0) << ", " << vector(1) << "\n";
    cout << angle*180/M_PI << "\n\n";
    return angle;
}

void flipVertices(vector<Vector2d>& vertices) {
	for (Vector2d& vertex: vertices) {
        vertex = Vector2d(-vertex(0), -vertex(1));
        cout << vertex(0) << ", " << vertex(1) << "\n";
    }
}

void MyClass::findMinkowskiDiff(const amp::Obstacle2D& obstacle, vector<Vector2d> robotVertices) {
    cout << "DING DONG\n";
    vector<Vector2d> obstacleVertices = obstacle.verticesCCW();
    flipVertices(robotVertices);
    vector<Vector2d> cSpaceObstacle;
    int n = obstacleVertices.size();
    int m = robotVertices.size();
    int i = 0;
    int j = 0;
    while (i < n + 1 && j < m + 1) {
        cSpaceObstacle.push_back(obstacleVertices[i] + robotVertices[j]);
        if (findAngle(obstacleVertices[i], obstacleVertices[i + 1]) < findAngle(robotVertices[j], robotVertices[j + 1])) {
            i++;
        }
        else if (findAngle(obstacleVertices[i], obstacleVertices[i + 1]) > findAngle(robotVertices[j], robotVertices[j + 1])) {
            j++;
        }
        else {
            i++;
            j++;
        }
        // if (i == n + 1 && j == m + 1) break;
    }
}