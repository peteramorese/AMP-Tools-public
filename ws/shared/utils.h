#include "AMPCore.h"
#include <Eigen/Core>
namespace utils {
    bool sensor(double epsilon, double theta, std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d point);
    Eigen::Vector2d raytrace(std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d q_init, Eigen::Vector2d q_goal, double epsilon = 1e-1);
    bool raycastObstacleDetection(const amp::Obstacle2D& obstacle, const Eigen::Vector2d& point);
    bool checkIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q1, const Eigen::Vector2d& q2);
};
