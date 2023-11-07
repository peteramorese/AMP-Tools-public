// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"

using namespace amp;

void problem1() {
    MultiAgentProblem2D problem = HW8::getWorkspace1();
    std::vector<double> stdVector;
    Eigen::VectorXd initState(problem.numAgents() * 2);
    int i = 0;
    for (const CircularAgentProperties& agent : problem.agent_properties) {
        stdVector.push_back(agent.radius);
        initState(2*i) = agent.q_init(0);
        initState(2*i + 1) = agent.q_init(1);
        i++;
    }
    Eigen::VectorXd eigenVector = Eigen::Map<Eigen::VectorXd>(stdVector.data(), stdVector.size());
    std::cout << "Eigen Vector:\n" << eigenVector << std::endl;
    std::cout << "State Vector:\n" << initState << std::endl;
    MyCentralPlanner planner(100, 1.0, 0.05);
    // MultiAgentPath2D path = planner.plan(problem);
}



void problem2() {
}


int main(int argc, char** argv) {
    problem1();
    // problem2();
    // Visualizer::showFigures();
    // HW7::grade<MyPRM, MyRRT>("yusif.razzaq@colorado.edu", argc, argv, std::make_tuple(500, 2, true), std::make_tuple(1000, 2, true));
    return 0;
}