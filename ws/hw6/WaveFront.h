#include "AMPCore.h"
#include "hw/HW6.h"
#include "CSpaceConstructor.h"

using namespace amp;
using std::vector, Eigen::Vector2d, std::pair, std::size_t;

struct MyNode {
    int value;
    pair<int, int> ind;
    pair<int, int> parent;
};

class MyPointWFAlgo : public amp::PointWaveFrontAlgorithm {
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& env) override {
            double cellWidth = 0.25;
            int cells0 = static_cast<int>(std::floor((env.x_max - env.x_min) / cellWidth));
            int cells1 = static_cast<int>(std::floor((env.y_max - env.y_min) / cellWidth));
            gridSize = {cells0, cells1};
            x0lim = {env.x_min, env.x_max};
            x1lim = {env.y_min, env.y_max};
            std::unique_ptr<CSpaceConstructor> cSpace = std::make_unique<CSpaceConstructor>(cells0, cells1, env.x_min, env.x_max, env.y_min, env.y_max);
            cSpace->populateGrid(env.obstacles);
            return cSpace;
        }

        // This is just to get grade to work, you DO NOT need to override this method
        // virtual amp::Path2D plan(const amp::Problem2D& problem) override { return amp::Path2D(); }

        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const GridCSpace2D& grid_cspace) override;
        void extendWave(const MyNode& cell, DenseArray2D<MyNode>& waveGrid);
        void findPath(const MyNode& finalCell, const DenseArray2D<MyNode>& waveGrid, amp::Path2D& path);
        void defineObstacles(const amp::GridCSpace2D& grid_cspace, DenseArray2D<MyNode>& waveGrid);
        Vector2d getPointFromCell(const std::pair<int, int>& cell);

    private:
        vector<MyNode> cellQueue;
        bool initReached;
        pair<size_t, size_t> initCell;
        pair<size_t, size_t> goalCell;
        pair<size_t, size_t> gridSize;
        pair<double, double> x0lim;
        pair<double, double> x1lim;
        // DenseArray2D<int> waveGrid;
};

class MyManipWFAlgo : public amp::ManipulatorWaveFrontAlgorithm {
    public:
        // Default ctor
        MyManipWFAlgo() {}
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyManipConstructor>()) {}
            // : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<CSpaceConstructor>(40, 40, 0, 10, 0, 10)) {  } 
            

        // You can have custom ctor params for all of these classes
        // MyManipWFAlgo(const amp::Environment2D& env) 
        //     : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<CSpaceConstructor>(40, 40, 0, 10, 0, 10)) {   
        // }

        virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const LinkManipulator2D& manipulator, const amp::Environment2D& env) {
            double cellWidth = 2*M_PI/50;
            double min = 0;
            double max = 2 * M_PI;
            x0lim = x1lim = {min, max};
            int cells = static_cast<int>(std::floor((max - min) / cellWidth));
            gridSize = {cells, cells};
            std::unique_ptr<CSpaceConstructor> cSpace = std::make_unique<CSpaceConstructor>(cells, cells, min, max, min, max);
            cSpace->populateGrid(manipulator.getLinkLengths(), env.obstacles);
            return cSpace;
        }

        // This is just to get grade to work, you DO NOT need to override this method
        virtual amp::ManipulatorTrajectory2Link plan(const LinkManipulator2D& link_manipulator_agent, const amp::Problem2D& problem) override {
            return amp::ManipulatorTrajectory2Link();
        }
        
        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const GridCSpace2D& grid_cspace) override;
        void extendWave(const MyNode& cell, DenseArray2D<MyNode>& waveGrid);
        void findPath(const MyNode& finalCell, const DenseArray2D<MyNode>& waveGrid, amp::Path2D& path);
        void defineObstacles(const amp::GridCSpace2D& grid_cspace, DenseArray2D<MyNode>& waveGrid);
        Vector2d getPointFromCell(const std::pair<int, int>& cell);
    private:
        vector<MyNode> cellQueue;
        bool initReached;
        pair<size_t, size_t> initCell;
        pair<size_t, size_t> goalCell;
        pair<size_t, size_t> gridSize;
        pair<double, double> x0lim;
        pair<double, double> x1lim;
};


