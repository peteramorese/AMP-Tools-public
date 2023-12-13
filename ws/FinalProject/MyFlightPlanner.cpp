#include "MyFlightPlanner.h"


using Node = uint32_t;

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
            if(!c.multDiskCollision2D(samples[minID].xy, q_rand, problem) && c.checkLOS(problem.numGA())){
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


