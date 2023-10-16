#include "AMPCore.h"
#include "hw/HW6.h"
using std::vector, Eigen::Vector2d;

class MyWaveFrontAlgorithm : amp::WaveFrontAlgorithm {
    public:
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override;
};

class MyPointWaveFrontAlgorithm : public amp::PointWaveFrontAlgorithm {
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override;
        // void populateGrid(const vector<double>& linkLengths, const vector<amp::Obstacle2D>& obstacles);
        // bool checkCollision(const Vector2d& prevJoint, const Vector2d& currJoint);
};

class MyManipulatorWaveFrontAlgorithm : public amp::ManipulatorWaveFrontAlgorithm {
    public:
        MyManipulatorWaveFrontAlgorithm(const std::shared_ptr<GridCSpace2DConstructor>& c_space_constructor) 
            : ManipulatorWaveFrontAlgorithm(c_space_constructor) {}
        virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override;
};


