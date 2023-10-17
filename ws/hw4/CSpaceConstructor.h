#include "AMPCore.h"
#include "HelpfulClass.h"
#include "hw/HW4.h"
using std::vector, Eigen::Vector2d;

class CSpaceConstructor : public amp::GridCSpace2D {
    public:
        CSpaceConstructor(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max);
        void populateGrid(const vector<amp::Obstacle2D>& obstacles);
        void populateGrid(const vector<double>& linkLengths, const vector<amp::Obstacle2D>& obstacles);
        bool checkCollision(const Vector2d& prevJoint, const Vector2d& currJoint);
        virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const override;
    private:
        vector<amp::Obstacle2D> allObstacles;
};

class GradingConstructor : public amp::GridCSpace2DConstructor {
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override {
            auto cSpace = std::make_unique<CSpaceConstructor>(100, 100, 0.0, 1.0, 0.0, 1.0);
            // CSpaceConstructor cSpace(50, 50, -10, 10, -10, 10);
            cSpace->populateGrid(manipulator.getLinkLengths(), env.obstacles);
            // CSpaceConstructor* pointer = &cSpace;
            return cSpace;
    };
};
