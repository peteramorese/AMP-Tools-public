#include "AMPCore.h"
#include "hw/HW6.h"
#include "../hw4/CSpaceConstructor.h"
using namespace amp;
using std::vector, Eigen::Vector2d, std::pair, std::size_t;

// class MyWaveFrontAlgorithm : amp::WaveFrontAlgorithm {
//     public:
//         virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override;
// };

struct Cell {
    pair<int, int> cell;
    Cell* parent;
};

class MyPointWFAlgo : public amp::PointWaveFrontAlgorithm {
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& env) override {
            int cells = 100;
            auto cSpace = std::make_unique<CSpaceConstructor>(cells, cells, 0, 10, 0, 10);
            cSpace->populateGrid(env.obstacles);
            return cSpace;
        }

        // This is just to get grade to work, you DO NOT need to override this method
        virtual amp::Path2D plan(const amp::Problem2D& problem) override {
            return amp::Path2D();
        }

        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override;
        void extendWave(const Cell& goalCell, const amp::GridCSpace2D& grid_cspace);
        amp::Path2D findPath(const Cell& finalCell);
    private:
        vector<Cell> cellQueue;
        bool initReached;
        pair<size_t, size_t> initCell;
        pair<size_t, size_t> goalCell;
};

// class MyManipWFAlgo : public amp::ManipulatorWaveFrontAlgorithm {
//     public:
//         // Default ctor
//         MyManipWFAlgo()
//             : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<CSpaceConstructor>()) {}

//         // You can have custom ctor params for all of these classes
//         MyManipWFAlgo(const std::string& beep) 
//             : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<CSpaceConstructor>()) {LOG("construcing... " << beep);}

//         // This is just to get grade to work, you DO NOT need to override this method
//         virtual amp::ManipulatorTrajectory2Link plan(const LinkManipulator2D& link_manipulator_agent, const amp::Problem2D& problem) override {
//             return amp::ManipulatorTrajectory2Link();
//         }
        
//         // You need to implement here
//         virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override {
//             return amp::Path2D();
//         }
// };


