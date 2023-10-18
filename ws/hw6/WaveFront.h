#include "AMPCore.h"
#include "hw/HW6.h"
#include "CSpaceConstructor.h"

using namespace amp;
using std::vector, Eigen::Vector2d, std::pair, std::size_t;

struct MyNode {
    pair<int, int> ind;
    MyNode* parent;
};

class MyPointWFAlgo : public amp::PointWaveFrontAlgorithm {
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& env) override {
            double cellWidth = 0.25;
            int cells0 = static_cast<int>(std::floor((env.x_max - env.x_min) / cellWidth));
            int cells1 = static_cast<int>(std::floor((env.y_max - env.y_min) / cellWidth));
            cout << cells0 << cells1 << "\n";
            gridSize = {cells0, cells1};
            std::unique_ptr<CSpaceConstructor> cSpace = std::make_unique<CSpaceConstructor>(cells0, cells1, env.x_min, env.x_max, env.y_min, env.y_max);
            cSpace->populateGrid(env.obstacles);
            return cSpace;
        }

        // This is just to get grade to work, you DO NOT need to override this method
        virtual amp::Path2D plan(const amp::Problem2D& problem) override { return amp::Path2D(); }

        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override;
        void extendWave(MyNode& cell);
        amp::Path2D findPath(const MyNode& finalCell);
        void defineObstacles(const amp::GridCSpace2D& grid_cspace);
    private:
        vector<MyNode> cellQueue;
        bool initReached;
        pair<size_t, size_t> initCell;
        pair<size_t, size_t> goalCell;
        pair<size_t, size_t> gridSize;
        int waveGrid[172][56];
};

// class MyManipWFAlgo : public amp::ManipulatorWaveFrontAlgorithm {
//     public:
//         // Default ctor
//         MyManipWFAlgo()
//             : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<CSpaceConstructor>()) {}

//         // You can have custom ctor params for all of these classes
//         MyManipWFAlgo(const std::string& beep) 
//             : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<CSpaceConstructor>(40, 40, 0, 10, 0, 10)) {LOG("construcing... " << beep);}

//         // This is just to get grade to work, you DO NOT need to override this method
//         virtual amp::ManipulatorTrajectory2Link plan(const LinkManipulator2D& link_manipulator_agent, const amp::Problem2D& problem) override {
//             return amp::ManipulatorTrajectory2Link();
//         }
        
//         // You need to implement here
//         virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override {
//             return amp::Path2D();
//         }
// };


