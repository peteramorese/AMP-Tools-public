#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiPlanner.h"
#include "HelpfulClass.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    MyCentralizedMultiAgentRRT cenRRT;
    MyDecentralizedMultiAgentRRT decRRT;
    HW8::grade(cenRRT, decRRT, "collin.hudson@colorado.edu", argc, argv);
    return 0;
}