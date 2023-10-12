// I'm going to remove this project eventually, but here is a simple example for how to visualize a potential function

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"


class MyPotentialFunction : public amp::PotentialFunction2D {
    public:
        virtual double operator()(const Eigen::Vector2d& q) const override {
            return q[0] * q[0] + q[1] * q[1];
        }
};

int main(int argc, char** argv) {
    amp::Visualizer::makeFigure(MyPotentialFunction{}, -10.0, 10.0, -10.0, 10.0, 20);
    amp::Visualizer::showFigures();
    return 0;
}