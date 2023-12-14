#include "MyFlightPlanner.h"


using Node = uint32_t;
// MyFlightPlanner member functions
amp::MultiAgentPath2D MyFlightPlanner::plan(const UASProblem& problem){
    // Centralized Planner parameters
    getN() = 50000; // number of tries
    getS() = 0.5;   // step size
    FlightChecker c;

    auto start = std::chrono::high_resolution_clock::now();

    // initialize LOS graph with empty edge sets for each Ground Agent and UAS
    c.makeLOS(problem);

    //{
        amp::MultiAgentPath2D path;
        std::vector<sampleS> samples;
        // Construct init super state
        Eigen::VectorXd init(2*problem.numAgents());
        for(int j = 0; j < 2*problem.numAgents(); j += 2){
            init(j) = problem.agent_properties[j/2].q_init(0);
            init(j + 1) = problem.agent_properties[j/2].q_init(1);
        }
        // LOG(init);
        samples.push_back(sampleS(init,-1));
        Eigen::VectorXd q_rand(3*problem.numAgents());
        bool soln = false;
        int minID = 0;
        double tempMin = 0;
        int tries = 0;
        int numNodes = 1;
        while(!soln && tries < getN()){
            //Generate q_rand
            int tst = amp::RNG::randi(0,100);
            for(int j = 0; j < 3*problem.numAgents(); j += 3){
                q_rand(j) = amp::RNG::randd(problem.x_min, problem.x_max); //x
                q_rand(j+1) = amp::RNG::randd(problem.y_min, problem.y_max); //y
                q_rand(j+2) = amp::RNG::randd(0, 2*M_PI); //heading angle (velocity i.e. groundspeed is constant)
            }
            //Find closest node in tree to q_rand
            minID = 0;
            tempMin = (samples[0].xy - q_rand).norm();
            for(int k = 1; k < samples.size(); k++){
                if((samples[k].xy - q_rand).norm() < tempMin){
                    tempMin = (samples[k].xy - q_rand).norm();
                    minID = k;
                }
            }
            splitAndStep2D(samples[minID].xy, q_rand, getS());
            c.updateLOS(samples[minID].xy, problem, numNodes);
            // q_rand = ((1 - stepSize/tempMin)*samples[minID].xy + (stepSize/tempMin)*q_rand);
            if(!c.multDiskCollision2D(samples[minID].xy, q_rand, problem) && c.checkLOS(problem.numGA)){
                // LOG("Adding state: " << q_rand);
                numNodes++;
                sampleS q_randS(q_rand,minID);
                samples.push_back(q_randS);
                if(numNodes >= problem.maxTime){
                    soln = true;
                }
            }
            tries++;
        }
        std::list<Eigen::VectorXd> l;
        if(soln){
            sampleS testS = samples.back();
            while(testS.back != -1){
                l.push_front(testS.xy);
                // LOG("Pushing to front state " << testS.xy);
                testS = samples[testS.back];
            }
            l.push_front(init);
            for(int j = 0; j < 3*problem.numAgents(); j += 3){
                amp::Path2D tempPath;
                for (std::list<Eigen::VectorXd>::iterator it=l.begin(); it != l.end(); ++it){
                    Eigen::VectorXd tempListEle = *it;
                    Eigen::Vector2d tempVec(tempListEle(j),tempListEle(j+1));
                    // LOG("Adding tempVec " << tempVec);
                    // LOG("From tempListEle " << tempListEle);
                    tempPath.waypoints.push_back(tempVec);
                }
                // LOG("Adding tempPath");
                path.agent_paths.push_back(tempPath);
                
            }
        }
        else{
            if(tries == getN()){
                LOG("Ran outta tries buddy :(");
            }else{
                LOG("Couldn't find path :(");
            }
        }
        getN() = numNodes;
        
    //}


    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = duration_cast<std::chrono::milliseconds>(stop - start);
    getT() = duration.count();
    return path;
}
// UASProblem constructor
UASProblem::UASProblem(uint32_t n_GA, uint32_t n_UAV, uint32_t n_Obs, double min_Obs, double max_Obs, double size_UAV){

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
    RRT.getS() = 0.5;
    for(int j = 0; j < this->numAgents(); j++){
        RRT.plan(randGen, tempPaths, j);
        endGAt.push_back(tempPaths.agent_paths[j].waypoints.size());
        if(tempPaths.agent_paths[j].waypoints.size() > maxTime){
            maxTime = tempPaths.agent_paths[j].waypoints.size();
        }
    }
    GApaths = tempPaths;
    amp::Visualizer::makeFigure(*this,GApaths);
    for(int j = 0; j < numGA; j++){
        LOG("GA[" << j << "], init: (" << GApaths.agent_paths[j].waypoints[0](0) << ","<< GApaths.agent_paths[j].waypoints[0](1) << "), path length: " << endGAt[j]);
    }
    // Generate initial positions of UAVs
    cSpec.n_agents = n_UAV;
    randGen = envGen.generateRandomMultiAgentProblem(eSpec, cSpec);
    this->agent_properties = randGen.agent_properties;
    FlightChecker c;
    c.makeLOS(*this);
    for(int j = 0; j < n_UAV; j++){
        Eigen::VectorXd tempInit = Eigen::VectorXd::Zero(3*n_UAV);
        do{
            for(int j = 0; j < 3*n_UAV; j += 3){
                tempInit(j) = amp::RNG::randd(this->x_min, this->x_max); //x
                tempInit(j+1) = amp::RNG::randd(this->y_min, this->y_max); //y
                tempInit(j+2) = amp::RNG::randd(0, 2*M_PI); //heading angle (velocity i.e. groundspeed is constant)
            }
            c.updateLOS(tempInit, *this, 0);
        }while(!c.checkLOS(n_UAV));
        this->agent_properties[j].radius = size_UAV;
        this->agent_properties[j].q_init = tempInit;

    }


}

// FlightChecker member functions
void FlightChecker::makeLOS(const UASProblem& problem){
    for(int j = 0; j < (problem.numGA + problem.numAgents()); j++){
        std::set<int> temp;
        for(int k = 0; k < (problem.numGA + problem.numAgents()); k++){
            if(j < k){
                // LOG(j << " and " << k << ", numGA= " << problem.numGA << ", numUAS= " << problem.numAgents());
                Eigen::Vector2d posj;
                Eigen::Vector2d posk;
                if(j < problem.numGA){
                    posj = problem.GApaths.agent_paths[j].waypoints[0];
                }
                else{
                    posj = problem.agent_properties[j].q_init;
                }
                if(k < problem.numGA){
                    posk = problem.GApaths.agent_paths[j].waypoints[0];
                }
                else{
                    posk = problem.agent_properties[k].q_init;
                }
                if(inLOS(posj,posk,problem)){
                    temp.insert(k);
                }
            }
        }
        losGraph.push_back(temp);
    }
}

void FlightChecker::updateLOS(Eigen::VectorXd state, const UASProblem& problem, int time){
    // Checks LOS between all ground agents and UAS, and ipdates connections stored in losGraph
    for(int j = 0; j < (problem.numGA + problem.numAgents()); j++){
        std::set<int> temp;
        for(int k = 0; k < (problem.numGA + problem.numAgents()); k++){
            if(j < k){
                Eigen::Vector2d posj;
                Eigen::Vector2d posk;
                if(j < problem.numGA){
                    posj = problem.GApaths.agent_paths[j].waypoints[time];
                }
                else{
                    posj(0) = state((j - problem.numGA)*3);
                    posj(1) = state((j - problem.numGA)*3 + 1);
                }
                if(k < problem.numGA){
                    posk = problem.GApaths.agent_paths[k].waypoints[time];
                }
                else{
                    posk(0) = state((k - problem.numGA)*3);
                    posk(1) = state((k - problem.numGA)*3 + 1);
                }
                if(inLOS(posj,posk,problem)){
                    temp.insert(k);
                }
            }
        }
        losGraph.push_back(temp);
    }

}

bool FlightChecker::checkLOS(int numGA){
    // LOG("Starting LOS Check");
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
    auto it = next(closedSet.begin(), numGA - 1);
    // LOG("LOS Check Complete");
    return *it == (numGA - 1);
}

