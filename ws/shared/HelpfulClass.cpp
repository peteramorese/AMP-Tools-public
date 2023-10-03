#include "HelpfulClass.h"

using std::vector, std::string, std::cout, Eigen::Vector2d;

double findAngle(const Vector2d& point1, const Vector2d& point2) {
    Vector2d vector = point2 - point1;
    double angle = std::atan2(vector.y(), vector.x());
    cout << angle << "\n";
}

void MyClass::findMinkowskiDiff(const amp::Obstacle2D& obstacle) {
    cout << "DING DONG\n";
    vector<Vector2d> obstacleVertices = obstacle.verticesCCW();
    vector<Vector2d> robotVertices = { Vector2d(0, 0) , Vector2d(1, 2), Vector2d(0, 2) };
    vector<Vector2d> cSpaceObstacle;
    int i = 0;
    int j = 0;
    while (int k = 0; k < obstacleVertices.size() + robotVertices.size(); ++k) {
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
    }
}