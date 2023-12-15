#include "MyFlightPlanner.h"


using Node = uint32_t;
// MyFlightPlanner member functions
void MyFlightPlanner::splitAndStep2D(Eigen::VectorXd state, Eigen::VectorXd& next, double stepSize){
    for(int j = 0; j < state.size(); j += 3){
        Eigen::Vector2i subIdx(j,j+1);
        Eigen::Vector2d temp = ((1 - stepSize/(state - next).norm()))*state(subIdx) + (stepSize/(state - next).norm())*next(subIdx);
        next(j) = temp(0);
        next(j + 1) = temp(1);
    }
}

amp::MultiAgentPath2D MyFlightPlanner::plan(const UASProblem& problem){
    // Centralized Planner parameters
    FlightChecker c;

    auto start = std::chrono::high_resolution_clock::now();

    // initialize LOS graph with empty edge sets for each Ground Agent and UAS
    c.makeLOS(problem);

    //{
        amp::MultiAgentPath2D path;
        if(!problem.initCond){
            return path;
        }
        std::vector<sampleS> samples;
        // Construct init super state
        Eigen::VectorXd init(3*problem.numAgents());
        for(int j = 0; j < 3*problem.numAgents(); j += 3){
            init(j) = problem.agent_properties[j/3].q_init(0);
            init(j + 1) = problem.agent_properties[j/3].q_init(1);
            init(j + 2) = 0;
        }
        // LOG(init);
        samples.push_back(sampleS(init,-1));
        Eigen::VectorXd q_rand(3*problem.numAgents());
        bool soln = false;
        int minID = 0;
        double tempMin = 0;
        int oldestID = 0;
        int oldestT = 0;
        int tries = 0;
        int numNodes = 1;
        // LOG("Ready to grow tree");
        while(!soln && tries < getN()){
            //Generate q_rand
            for(int j = 0; j < 3*problem.numAgents(); j += 3){
                q_rand(j) = amp::RNG::randd(problem.x_min, problem.x_max); //x
                q_rand(j+1) = amp::RNG::randd(problem.y_min, problem.y_max); //y
                q_rand(j+2) = amp::RNG::randd(0, 2*M_PI); //heading angle (velocity i.e. groundspeed is constant)
            }
            // LOG("Got a sample");
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
            c.updateLOS(samples[minID].xy, problem, samples[minID].tFromInit + 1);
            // q_rand = ((1 - stepSize/tempMin)*samples[minID].xy + (stepSize/tempMin)*q_rand);
            if(!c.multDiskCollision2D(samples[minID].xy, q_rand, problem) && c.checkLOS(problem.numGA)){
                // LOG("Adding state: " << q_rand << " minID: " << minID << ", maxTime: " << problem.maxTime);
                numNodes++;
                sampleS q_randS(q_rand,minID,samples[minID].tFromInit + 1);
                samples.push_back(q_randS);
                if(q_randS.tFromInit >= problem.maxTime){
                    soln = true;
                }
                else if(q_randS.tFromInit >= oldestT){
                    oldestT = q_randS.tFromInit;
                    oldestID = minID;
                }
            }
            tries++;
            if((tries % 5000) == 0){
                LOG("tries for samples: " << tries << ", samples found: " << numNodes);
            }
        }
        // LOG("out of while loop");
        std::list<Eigen::VectorXd> l;
        if(soln){
            sampleS testS = samples.back();
            // LOG("Popping state: " << q_rand << " tfrominit: " << testS.tFromInit << ", maxTime: " << problem.maxTime);
            while(testS.back != -1){
                l.push_front(testS.xy);
                // LOG("Pushing to front state " << testS.xy);
                testS = samples[testS.back];
            }
            l.push_front(init);
            // LOG("ready to split");
            for(int j = 0; j < 3*problem.numAgents(); j += 3){
                amp::Path2D tempPath;
                for (std::list<Eigen::VectorXd>::iterator it=l.begin(); it != l.end(); ++it){
                    Eigen::VectorXd tempListEle = *it;
                    Eigen::Vector2d tempVec(tempListEle(j),tempListEle(j+1));
                    // LOG("Adding tempVec " << tempVec);
                    // LOG("From tempListEle " << tempListEle);
                    tempPath.waypoints.push_back(tempVec);
                }
                path.agent_paths.push_back(tempPath);
                // LOG("Added tempPath");
                
            }
        }
        else{
            if(tries == getN()){
                LOG("Ran outta tries buddy :(");
                LOG("Longest path: " << oldestT);
                sampleS testS = samples[oldestID];
                while(testS.back != -1){
                    l.push_front(testS.xy);
                    testS = samples[testS.back];
                }
                l.push_front(init);
                for(int j = 0; j < 3*problem.numAgents(); j += 3){
                    amp::Path2D tempPath;
                    for (std::list<Eigen::VectorXd>::iterator it=l.begin(); it != l.end(); ++it){
                        Eigen::VectorXd tempListEle = *it;
                        Eigen::Vector2d tempVec(tempListEle(j),tempListEle(j+1));
                        tempPath.waypoints.push_back(tempVec);
                    }
                    path.agent_paths.push_back(tempPath);                    
                }
            }else{
                LOG("Couldn't find path :(");
            }

        }
        getN() = numNodes;
        
    //}


    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = duration_cast<std::chrono::milliseconds>(stop - start);
    getT() = duration.count();
    // LOG("ready to return");
    return path;
}
// UASProblem constructor
UASProblem::UASProblem(uint32_t n_GA, uint32_t n_UAV, uint32_t n_Obs, double min_Obs, double max_Obs, double size_UAV, double los_dist){
    losLim = los_dist;
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
    Eigen::VectorXd tempInit = Eigen::VectorXd::Zero(3*n_UAV);
    Eigen::Vector2d tempXY = Eigen::Vector2d::Zero();
    bool collision = false;
    int tries = 0;
    //Generate safe meta state that is free from collisions and allows LOS communication between all GAs
    amp::MultiAgentPath2D losPath; //position of states for all times that enable LOS
    std::list<Eigen::VectorXd> l;
    amp::CircularAgentProperties UAV;
    UAV.radius = size_UAV;
    for(int t = 0; t < maxTime; t++){
        do{
            collision = false;
            for(int j = 0; j < 3*n_UAV; j += 3){
                
                tempInit(j) = amp::RNG::randd(this->x_min, this->x_max); //x
                tempInit(j+1) = amp::RNG::randd(this->y_min, this->y_max); //y
                tempXY(0) = tempInit(j);
                tempXY(1) = tempInit(j+1);
                if(c.diskCollision2D(tempXY,UAV,*this)){
                    collision = true;
                }
                tempInit(j+2) = amp::RNG::randd(0, 2*M_PI); //heading angle (velocity i.e. groundspeed is constant)
            }
            c.updateLOS(tempInit, *this, t);
            tries++;
            // LOG("looking for init...");
        }while((!c.checkLOS(numGA) || collision) && tries < 1000);
        if(tries >= 1000){
            LOG("Can't find viable initial condition!");
            initCond = false;
        }
        else{
            //Split up meta state into each UAV's initial xy.
            // for(int j = 0; j < n_UAV; j++){
            //     this->agent_properties[j].radius = size_UAV;
            //     tempXY(0) = tempInit(3*j);
            //     tempXY(1) = tempInit(3*j+1);
            //     this->agent_properties[j].q_init = tempXY;
            //     LOG("init for UAV " << j << ": " << this->agent_properties[j].q_init);
            // }
            // //Logging stuff 
            // LOG("t = " << t);
            // c.printLOS();
            // Eigen::Vector2d g1;
            // Eigen::Vector2d g0;
            // if(t < this->GApaths.agent_paths[1].waypoints.size()){
            //     g1 = this->GApaths.agent_paths[1].waypoints[t];
            // }
            // else{
            //     g1 = this->GApaths.agent_paths[1].waypoints.back();
            // }
            // if(t < this->GApaths.agent_paths[0].waypoints.size()){
            //     g0 = this->GApaths.agent_paths[0].waypoints[t];
            // }
            // else{
            //     g0 = this->GApaths.agent_paths[0].waypoints.back();
            // }
            // LOG("LOS?: " << c.inLOS(g0, g1,*this));
            // std::cout <<", dist from " << 0 << " to " << 1 << " = " << (g0 - g1).norm() << std::endl;
            l.push_back(tempInit);
        }
    }
    if(initCond){
        for(int j = 0; j < 3*this->numAgents(); j += 3){
            amp::Path2D tempPath;
            int t = 0;
            for (std::list<Eigen::VectorXd>::iterator it=l.begin(); it != l.end(); ++it){
                Eigen::VectorXd tempListEle = *it;
                Eigen::Vector2d tempVec(tempListEle(j),tempListEle(j+1));
                LOG("Adding tempVec " << tempVec << " at t = " << t);
                // for(int g = 0; g < numGA; g++){
                //     Eigen::Vector2d g1;
                //     Eigen::Vector2d g0;
                //     if(t < this->GApaths.agent_paths[g].waypoints.size()){
                //         g1 = this->GApaths.agent_paths[g].waypoints[t];
                //     }
                //     else{
                //         g1 = this->GApaths.agent_paths[g].waypoints.back();
                //     }
                    
                //     std::cout <<", dist to " << g << " = " << (g1 - tempVec).norm();
                //     if(g > 0){
                //         if(t < this->GApaths.agent_paths[g-1].waypoints.size()){
                //             g0 = this->GApaths.agent_paths[g-1].waypoints[t];
                //         }
                //         else{
                //             g0 = this->GApaths.agent_paths[g-1].waypoints.back();
                //         }
                //         std::cout <<", dist from " << g - 1 << " to " << g << " = " << (g0 - g1).norm();
                //     }
                // }
                // std::cout << std::endl;
                // LOG("From tempListEle " << tempListEle);
                tempPath.waypoints.push_back(tempVec);
                t++;
            }
            losPath.agent_paths.push_back(tempPath);

            // LOG("Added tempPath");
        }
        amp::Visualizer::makeFigure(*this,losPath);
    }
    else{
        LOG("FAILED TO FIND LOS FOR ALL TIMES");
    }
}

// FlightChecker member functions
void FlightChecker::makeLOS(const UASProblem& problem){
    for(int j = 0; j < (problem.numGA + problem.numAgents()); j++){
        std::set<int> temp;
        for(int k = 0; k < (problem.numGA + problem.numAgents()); k++){
            if(j != k){
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

void FlightChecker::updateLOS(Eigen::VectorXd state, const UASProblem& problem, int time){
    // Checks LOS between all ground agents and UAS, and updates connections stored in losGraph
    // auto start = std::chrono::high_resolution_clock::now();
    losGraph.clear();
    std::set<int> temp;
    for(int j = 0; j < (problem.numGA + problem.numAgents()); j++){
        temp.clear();
        for(int k = 0; k < (problem.numGA + problem.numAgents()); k++){
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
    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = duration_cast<std::chrono::milliseconds>(stop - start);
    // LOG("Time to update LOS: " << duration.count() << "ms");

}

bool FlightChecker::checkLOS(int numGA){
    std::set<int> openSet;
    std::set<int> closedSet;
    openSet.insert(0);
    // printLOS();
    // LOG("start check");
    while(!openSet.empty()){
        //add top element of open set to closed set, and erase from open set
        int top = *openSet.begin();
        closedSet.insert(top);
        openSet.erase(openSet.begin());
        // LOG("popping " << top);
        //Add children of top to open set if not in closed set
        for (std::set<int>::iterator it=losGraph[top].begin(); it!=losGraph[top].end(); ++it){
            if(closedSet.find(*it) == closedSet.end()){
                openSet.insert(*it);
                // LOG("adding to open set " << *it);
            }
        }
    }
        // LOG("closed set");
        // for (auto con : closedSet)
        // {
        //     std::cout << con << ' ';
        // }
        // std::cout << std::endl;
    // check if closed set has numGA ground agents connected
    if(closedSet.size() > (numGA - 1)){
        auto it = next(closedSet.begin(), numGA - 1);
        //returns true if all ground agents are connected to eachother
        // LOG("returns " << (*it == (numGA - 1)));
        return (*it == (numGA - 1));
    }
    else{
        // LOG("returns false");
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

