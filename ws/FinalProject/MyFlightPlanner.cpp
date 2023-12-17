#include "MyFlightPlanner.h"


// MyFlightPlanner member functions
amp::MultiAgentPath2D MyFlightPlanner::plan(UASProblem& problem){
    FlightChecker c;
    c.makeLOS(problem);
    Eigen::VectorXd targetState = Eigen::VectorXd::Zero(3*problem.numUAV);
    Eigen::VectorXd tempXY = Eigen::VectorXd::Zero(2*problem.numUAV); //no heading angle included
    Eigen::VectorXd lastState = targetState;
    Eigen::VectorXd lastXY = tempXY;
    bool collision = false;
    int tries = 0;
    //Generate safe meta state that is free from collisions and allows LOS communication between all GAs
    amp::MultiAgentPath2D losPath; //position of states for all times that enable LOS
    std::list<Eigen::VectorXd> l;
    amp::CircularAgentProperties UAV;
    UAV.radius = problem.radUAV;
    bool soln = true;
    for(int t = 0; t < problem.maxTime; t++){
        do{
            collision = false;
            // Generate target configuration
            for(int j = 0; j < 3*problem.numUAV; j += 3){
                if(t == 0){
                    targetState(j) = amp::RNG::randd(problem.x_min, problem.x_max); //x
                    targetState(j+1) = amp::RNG::randd(problem.y_min, problem.y_max); //y
                }
                else{
                    targetState(j) = amp::RNG::randd(std::max(lastState(j) - problem.connectRadius,problem.x_min), std::min(lastState(j) + problem.connectRadius,problem.x_max));
                    targetState(j+1) = amp::RNG::randd(std::max(lastState(j+1) - problem.connectRadius,problem.y_min), std::min(lastState(j+1) + problem.connectRadius,problem.y_max));
                }
                tempXY(2*(j/3)) = targetState(j);
                tempXY(2*(j/3)+1) = targetState(j+1);
                targetState(j+2) = amp::RNG::randd(0, 2*M_PI); //heading angle (velocity i.e. groundspeed is constant)
            }
            c.updateLOS(targetState, problem, t);
            if(t == 0){
                collision = c.multDiskDiskCollision2D(tempXY,problem);
            }
            else{
                collision = c.multDiskCollision2D(lastXY,tempXY,problem);
                //If checkLOS good, randomly sample control inputs from past state to present state
                if(kino){
                    if(c.checkLOS(problem.numGA)){
                        if(!propState(lastState, targetState, problem, c)){
                            collision = true;
                        }
                        else{
                            c.updateLOS(targetState, problem, t);
                        }
                    }
                    else{collision = true;}
                }
            }
            tries++;            
        }while((!c.checkLOS(problem.numGA) || collision) && tries < getN());
        if(tries >= getN()){
            LOG("Can't find viable state at t=" << t);
            soln = false;
            break;
        }
        else{
            // c.printLOS();
            l.push_back(targetState);
            lastState = targetState;
            lastXY = tempXY;
        }
    }
    if(soln){
        for(int j = 0; j < 3*problem.numUAV; j += 3){
            amp::Path2D tempPath;
            int t = 0;
            for (std::list<Eigen::VectorXd>::iterator it=l.begin(); it != l.end(); ++it){
                Eigen::VectorXd tempListEle = *it;
                Eigen::Vector2d tempVec(tempListEle(j),tempListEle(j+1));
                // LOG("Adding tempVec " << tempVec << " at t = " << t);
                tempPath.waypoints.push_back(tempVec);
                t++;
            }
            problem.agent_properties[j/3].q_init = tempPath.waypoints.front();
            problem.agent_properties[j/3].q_goal = tempPath.waypoints.back();
            losPath.agent_paths.push_back(tempPath);
            
        }
        success = true;
        return losPath;
    }
    else{
        // LOG("FAILED TO FIND LOS FOR ALL TIMES");
        success = false;
        return losPath;
    }
}

void MyFlightPlanner::makeFlightPlan(int minUAV, int maxUAV, int runs, UASProblem& problem){
    for(int n = minUAV; n <= maxUAV; n++){
        problem.changeNumUAV(n);
        // LOG("Attempting to plan with " << n << " UAVs");
        for(int k = 0; k < runs; k++){
            amp::MultiAgentPath2D solution = plan(problem);
            if(success){
                // amp::Visualizer::makeFigure(problem,solution);
                // LOG("Successful plan found using " << n << " UAVs");
                return;
            }
        }
    }
    LOG("Planner could not find a solution :(");
}

bool MyFlightPlanner::propState(Eigen::VectorXd& lastState, Eigen::VectorXd& nextState, const UASProblem& problem, FlightChecker& c){
    //Use random control inputs to move from lastState to a state near or at targetState, new state must be free from collisions
    Eigen::VectorXd tempState = Eigen::VectorXd::Zero(3*problem.numUAV);
    double v = 0;
    double w = 0;
    bool soln; //bool for if agent found a valid path
    Eigen::Vector2d tempXY = Eigen::Vector2d::Zero();
    Eigen::Vector2d nextXY = Eigen::Vector2d::Zero();
    Eigen::Vector2d closestXY(-99,-99);
    for(int j = 0; j < 3*problem.numUAV; j += 3){
        soln = false;
        closestXY(0) = -99;
        closestXY(1) = -99;
        for(int k = 0; k < 50; k++){
            if(!soln){
                v = amp::RNG::randd(problem.vMin, problem.vMax);
                w = amp::RNG::randd(problem.wMin, problem.wMax);
                // Unicycle and Euler Approximation for UAV j
                tempXY(0) = lastState(j) + v*cos(lastState(j + 2));
                tempXY(1) = lastState(j+1) + v*sin(lastState(j + 2));
                nextXY(0) = nextState(j);
                nextXY(1) = nextState(j+1);
                //check if valid state and compare Euler result state to target state
                if((tempXY(0) >= problem.x_min) && (tempXY(0) <= problem.x_max) &&
                 (tempXY(1) >= problem.y_min) && (tempXY(1) <= problem.y_max)){
                    if(!c.diskCollision2D(tempXY, problem.agent_properties[j/2],problem) && 
                     ((tempXY - nextXY).norm() < (closestXY - nextXY).norm())){
                        tempState(j) = tempXY(0);
                        tempState(j + 1) = tempXY(1);
                        tempState(j + 2) = lastState(j+2) + w;
                        closestXY(0) = tempXY(0);
                        closestXY(1) = tempXY(1);
                        soln = true;
                    }
                }
                
            }
        }
        if(!soln){
            return false;
        }
        
    }
    // LOG("target: " << nextState << " achieved: " << tempState);
    nextState = tempState;
    return true;
}

// UASProblem constructor
UASProblem::UASProblem(uint32_t n_GA, uint32_t n_UAV, uint32_t n_Obs, double min_Obs, double max_Obs, double size_UAV, double los_dist, double conRad){
    losLim = los_dist;
    connectRadius = conRad;
    amp::Random2DEnvironmentSpecification eSpec;
    eSpec.n_obstacles = n_Obs;
    eSpec.min_obstacle_region_radius = min_Obs;
    eSpec.max_obstacle_region_radius = max_Obs;
    amp::RandomCircularAgentsSpecification cSpec;
    cSpec.n_agents = n_GA;
    numGA = n_GA;
    amp::EnvironmentTools envGen;
    amp::MultiAgentProblem2D randGen = envGen.generateRandomMultiAgentProblem(eSpec, cSpec);
    this->obstacles = randGen.obstacles;
    this->agent_properties = randGen.agent_properties;
    // Solve multiagent paths for Ground Agents to generate paths to plan around using decentralized RRT planner
    amp::MultiAgentPath2D tempPaths;
    MyGoalBiasRRTND RRT;
    RRT.getN() = 50000;
    RRT.getS() = 1.5;
    for(int j = 0; j < this->numAgents(); j++){
        RRT.plan(randGen, tempPaths, j);
        // endGAt.push_back(tempPaths.agent_paths[j].waypoints.size());
        if(tempPaths.agent_paths[j].waypoints.size() > maxTime){
            maxTime = tempPaths.agent_paths[j].waypoints.size();
        }
        
    }
    GApaths = tempPaths;
    amp::Visualizer::makeFigure(*this,GApaths);
    for(int j = 0; j < numGA; j++){
        LOG("GA[" << j << "], init: (" << GApaths.agent_paths[j].waypoints[0](0) << ","<< GApaths.agent_paths[j].waypoints[0](1) << "), path length: " << GApaths.agent_paths[j].waypoints.size());
    }
    radUAV = size_UAV;
    numUAV = n_UAV;
    cSpec.n_agents = n_UAV;
    cSpec.min_agent_radius = radUAV;
    cSpec.max_agent_radius = radUAV;
    randGen = envGen.generateRandomMultiAgentProblem(eSpec, cSpec); //set the agent properties of problem to number of UAS
    this->agent_properties = randGen.agent_properties;
    vMin = 0.1; //min ground speed
    vMax = connectRadius; //max ground speed
    wMin = -0.5; //min angular velocity [rad/time]
    wMax = 0.5; //max angular velocity [rad/time]
}

void UASProblem::changeNumUAV(int n_UAV){
    //Updates number of agents in the problem to match the number of UAVs (necessary for plotting)
    numUAV = n_UAV;
    amp::EnvironmentTools envGen;
    amp::Random2DEnvironmentSpecification eSpec;
    amp::RandomCircularAgentsSpecification cSpec;
    cSpec.n_agents = n_UAV;
    cSpec.min_agent_radius = radUAV;
    cSpec.max_agent_radius = radUAV;
    amp::MultiAgentProblem2D randGen = envGen.generateRandomMultiAgentProblem(eSpec, cSpec); //set the agent properties of problem to number of UAS
    this->agent_properties = randGen.agent_properties;

}

// FlightChecker member functions
void FlightChecker::makeLOS(const UASProblem& problem){
    // checks the LOS between all the agents, essentially just instantiating the graph.
    // objects are rdifferentiated by their index, with all the GAs being added first, followed by all the UAVs
    for(int j = 0; j < (problem.numGA + problem.numUAV); j++){
        std::set<int> temp;
        for(int k = 0; k < (problem.numGA + problem.numUAV); k++){
            if(j != k){
                Eigen::Vector2d posj;
                Eigen::Vector2d posk;
                if(j < problem.numGA){
                    posj = problem.GApaths.agent_paths[j].waypoints[0];
                }
                else{
                    posj = problem.agent_properties[j].q_init;
                }
                if(k < problem.numGA){
                    posk = problem.GApaths.agent_paths[k].waypoints[0];
                }
                else{
                    posk = problem.agent_properties[k].q_init;
                }
                if(inLOS(posj,posk,problem) && ((posj - posk).norm() <= problem.losLim)){
                    temp.insert(k);
                }
            }
        }
        losGraph.push_back(temp);
    }
}

void FlightChecker::updateLOS(Eigen::VectorXd& state, const UASProblem& problem, int time){
    // Checks LOS between all ground agents and UAS, and updates connections stored in losGraph
    losGraph.clear();
    std::set<int> temp;
    for(int j = 0; j < (problem.numGA + problem.numUAV); j++){
        temp.clear();
        for(int k = 0; k < (problem.numGA + problem.numUAV); k++){
            if(j != k){
                Eigen::Vector2d posj;
                Eigen::Vector2d posk;
                if(j < problem.numGA){
                    if(time < problem.GApaths.agent_paths[j].waypoints.size()){
                        posj = problem.GApaths.agent_paths[j].waypoints[time];
                    }
                    else{
                        posj = problem.GApaths.agent_paths[j].waypoints.back();
                    }
                }
                else{
                    posj(0) = state((j - problem.numGA)*3);
                    posj(1) = state((j - problem.numGA)*3 + 1);
                }
                if(k < problem.numGA){
                    if(time < problem.GApaths.agent_paths[k].waypoints.size()){
                        posk = problem.GApaths.agent_paths[k].waypoints[time];
                    }
                    else{
                        posk = problem.GApaths.agent_paths[k].waypoints.back();
                    }
                }
                else{
                    posk(0) = state((k - problem.numGA)*3);
                    posk(1) = state((k - problem.numGA)*3 + 1);
                }
                if(inLOS(posj,posk,problem) && ((posj - posk).norm() <= problem.losLim) ){
                    temp.insert(k);
                }
            }
        }
        losGraph.push_back(temp);
    }

}

bool FlightChecker::checkLOS(int numGA){
    // Breadth first search that checks for connection to all the ground agents
    std::set<int> openSet;
    std::set<int> closedSet;
    openSet.insert(0);
    while(!openSet.empty()){
        //add top element of open set to closed set, and erase from open set
        int top = *openSet.begin();
        closedSet.insert(top);
        openSet.erase(openSet.begin());
        //Add children of top to open set if not in closed set
        for (std::set<int>::iterator it=losGraph[top].begin(); it!=losGraph[top].end(); ++it){
            if(closedSet.find(*it) == closedSet.end()){
                openSet.insert(*it);
            }
        }
    }

    // check if closed set has numGA ground agents connected
    if(closedSet.size() > (numGA - 1)){
        auto it = next(closedSet.begin(), numGA - 1);
        //returns true if all ground agents are connected to eachother
        return (*it == (numGA - 1));
    }
    else{
        return false;
    }
}

void FlightChecker::printLOS(){
    for(int j = 0; j < losGraph.size(); j++){
        LOG("idx: " << j);
        for (auto con : losGraph[j])
        {
            std::cout << con << ' ';
        }
        std::cout << std::endl;
    }
}

