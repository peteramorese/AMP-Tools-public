#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiPlanner.h"
#include "HelpfulClass.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    MyCentralizedMultiAgentRRT cenRRT;
    amp::MultiAgentProblem2D cenProb = HW8::getWorkspace1();
    amp::MultiAgentPath2D cenPath = cenRRT.plan(cenProb);
    HW8::check(cenPath, cenProb);
    return 0;
}