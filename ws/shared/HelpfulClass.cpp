#include "HelpfulClass.h"

amp::MultiAgentPath2D MyGoalBiasRRTND::plan(const amp::MultiAgentProblem2D& problem){
    auto start = std::chrono::high_resolution_clock::now();
    amp::MultiAgentPath2D path;
    std::vector<sampleS> samples;
    // Construct init and goal super states
    Eigen::VectorXd init;
    Eigen::VectorXd goal;
    for(int j = 0; j < 2*problem.numAgents(); j += 2){
        init(j) = problem.agent_properties[j/2].q_init(0);
        init(j + 1) = problem.agent_properties[j/2].q_init(1);
        goal(j) = problem.agent_properties[j/2].q_goal(0);
        goal(j + 1) = problem.agent_properties[j/2].q_goal(1);
    }
    samples.push_back(sampleS(init,-1));
    Eigen::VectorXd q_rand;
    bool soln = false;
    int minID = 0;
    int nodeNum = 1;
    double tempMin = 0;
    checkPath c;
    int steps = 0;
    while(!soln && steps < numIterations){
        //Generate q_rand
        int tst = amp::RNG::randi(0,100);
        if(tst > int(goalBiasP*100)){
            for(int j = 0; j < 2*problem.numAgents(); j += 2){
                q_rand(j) = amp::RNG::randd(problem.x_min, problem.x_max);
                q_rand(j+1) = amp::RNG::randd(problem.y_min, problem.y_max);
            }
        }
        else{
            // LOG("choosing goal at step " << steps << " since tst = " << tst << " and cutoff is " << int(goalBiasP*100));
            q_rand = goal;
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
        q_rand = ((1 - stepSize*problem.numAgents()/tempMin)*samples[minID].xy + (stepSize*problem.numAgents()/tempMin)*q_rand);
        if(!c.multDiskCollision2D(q_rand, samples[minID].xy, problem)){
            sampleS q_randS(q_rand,minID);
            samples.push_back(q_randS);
            nodeNum++;
            if((q_rand - goal).norm() < eps*problem.numAgents()){
                soln = true;
            }
        }
        steps++;
    }
    std::list<Eigen::Vector2d> l;
    if(soln){
        sampleS testS(goal,samples.size() - 1);
        while(testS.back != -1){
            l.push_front(testS.xy);
            testS = samples[testS.back];
        }
        l.push_front(init);
        for(int j = 0; j < 2*problem.numAgents(); j += 2){
            amp::Path2D tempPath;
            for (std::list<Eigen::Vector2d>::iterator it=l.begin(); it != l.end(); ++it){
                Eigen::Vector2d tempListEle = *it;
                Eigen::Vector2d tempVec(tempListEle(j),tempListEle(j+1));
                tempPath.waypoints.push_back(tempVec);
            }
            path.agent_paths.push_back(tempPath);
            
        }
    }
    else{
        if(steps == numIterations){
            LOG("Ran outta steps buddy :(");
        }else{
            LOG("Couldn't find path :(");
        }
        // path.waypoints.push_back(init);
        // path.waypoints.push_back(problem.q_goal);
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = duration_cast<std::chrono::milliseconds>(stop - start);
    time = duration.count();
    return path;


}

std::map<Node, Eigen::Vector2d> MyGoalBiasRRTND::makeMap(std::vector<Eigen::Vector2d> samples){
    std::map<Node, Eigen::Vector2d> mapOfSamples;
    for (Node j = 0; j < samples.size(); j++){ 
        mapOfSamples.insert({j, samples[j]}); 
    }
    return mapOfSamples;
}