#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include <Eigen/LU>

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        Eigen::Vector2d bugXY;

        Eigen::Vector2d bugNext;

        double bugStep = 0.01;

        amp::Polygon currentOb;

        int vertIdx;

        Eigen::Vector2d wallPoint;

        Eigen::Vector2d qLeave;
        
        double qLScore;
        
        bool hit = false;

        bool followMode;

        int limit = 50000;

        bool kill = false;

        double lastCornerDist;

        
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Add any other methods here...
        
        Eigen::Vector2d getNext(Eigen::Vector2d bugXY, Eigen::Vector2d goal, double stepSize){
            Eigen::Vector2d nextXY;
            double ySign = (1 - 2*(goal(1) - bugXY(1) < 0));
            double xSign = (1 - 2*(goal(0) - bugXY(0) < 0));
            //std::cout << "ySign: " << ySign << "xSign: " << xSign << std::endl;
            //If line to goal is vertical, move vertically by step size
            if((goal(0) - bugXY(0)) == 0){
                nextXY << bugXY(0), bugXY(1) + stepSize*ySign;
            }
            //If line to goal is horizontal, move horizontally by step size
            else if((goal(1) - bugXY(1)) == 0){
                nextXY << bugXY(0) + stepSize*xSign, bugXY(1);
            }
            else{
                //double sign = (1 - 2*(goal(0) - bugXY(0) < 0));
                //nextXY << bugXY(0) + stepSize*(xSign), (goal(1) - bugXY(1))/(goal(0) - bugXY(0))*(bugXY(0) + stepSize*(xSign));
                double theta = atan2((goal(1) - bugXY(1)),(goal(0) - bugXY(0)));
                //std::cout << "theta: " << theta << " cos(theta) = " << cos(theta) << " sin(theta) = "<< sin(theta) << std::endl;
                nextXY << bugXY(0) + cos(theta)*stepSize, bugXY(1) + sin(theta)*stepSize;
            }
            return nextXY;
        }
        Eigen::Vector2d getNextFollow(Eigen::Vector2d bugXY, Eigen::Vector2d obV1, Eigen::Vector2d obV2, double stepSize){
            // obV2 must be clockwise next from obV1!
            Eigen::Vector2d nextXY;
            double ySign = (1 - 2*(obV2(1) - obV1(1) < 0));
            double xSign = (1 - 2*(obV2(0) - obV1(0) < 0));
            //If wall to follow is vertical, move vertically by step size
            if((obV2(0) - obV1(0)) == 0){
                nextXY << bugXY(0), bugXY(1) + stepSize*ySign;
            }
            //If wall to follow is horizontal, move horizontally by step size
            else if((obV2(1) - obV1(1)) == 0){
                nextXY << bugXY(0) + stepSize*xSign, bugXY(1);
            }
            else{
                //nextXY << bugXY(0) + stepSize*(xSign), (obV2(1) - obV1(1))/(obV2(0) - obV1(0))*(bugXY(0) + stepSize*(xSign));
                double theta = atan2((obV2(1) - obV1(1)),(obV2(0) - obV1(0)));
                //std::cout << "theta: " << theta << std::endl;
                nextXY << bugXY(0) + cos(theta)*stepSize, bugXY(1) + sin(theta)*stepSize;
            }
            return nextXY;
        }
        Eigen::Vector2d getNextCorner(Eigen::Vector2d bugXY, Eigen::Vector2d corner, double theta){
            // nextPoint must be clockwise next from corner!
            Eigen::Vector2d nextXY;
            //nextXY << corner(0) + bugStep*sin(theta), corner(1) + bugStep*cos(theta);
            nextXY << corner(0) + bugStep*cos(theta), corner(1) + bugStep*sin(theta);
            return nextXY;
        }
        double getT(Eigen::Vector2d bugXY, Eigen::Vector2d bugNext, Eigen::Vector2d v1, Eigen::Vector2d v2){
            Eigen::Matrix2d Tmat1;
            Eigen::Matrix2d Tmat2;
            Tmat1.col(0) = bugXY-v1;
            Tmat1.col(1) = v1-v2;
            Tmat2.col(0) = bugXY-bugNext;
            Tmat2.col(1) = v1-v2;
            return Tmat1.determinant()/Tmat2.determinant();
        }
        double getU(Eigen::Vector2d bugXY, Eigen::Vector2d bugNext, Eigen::Vector2d v1, Eigen::Vector2d v2){
            Eigen::Matrix2d Tmat1;
            Eigen::Matrix2d Tmat2;
            Tmat1.col(0) = bugXY-v1;
            Tmat1.col(1) = bugXY-bugNext;
            Tmat2.col(0) = bugXY-bugNext;
            Tmat2.col(1) = v1-v2;
            return Tmat1.determinant()/Tmat2.determinant();
        }
        bool evalTU( double t, double u){
            return t >= 0 && t <= 1 && u >= 0 && u <= 1;
        }
        Eigen::Vector2d getHit(Eigen::Vector2d bugXY, Eigen::Vector2d bugNext, double t){
            Eigen::Vector2d interP;
            interP << bugXY(0) + t*(bugNext(0) - bugXY(0)), bugXY(1) + t*(bugNext(1) - bugXY(1));
            return interP;
        }
        void stepCheck(Eigen::Vector2d position, Eigen::Vector2d next, const amp::Problem2D& problem){
            hit = false;
            for(auto Ob : problem.obstacles){
                if(!hit){
                    for(int j = 0; j < Ob.verticesCCW().size(); j++){
                        if(!hit){
                            hit = checkHit(position,next,Ob,j); //hitPoint set in checkHit function
                        }
                    }
                }
            }
        }
        bool checkHit(Eigen::Vector2d position, Eigen::Vector2d next,amp::Polygon Ob, int startVert){
            int endVert;
            std::vector<Eigen::Vector2d> vertCW = Ob.verticesCW();
            int numV = vertCW.size() - 1;
            (startVert == 0) ? (endVert = numV) : endVert = startVert - 1;
            double t = getT(position,next,vertCW[startVert],vertCW[endVert]);
            double u = getU(position,next,vertCW[startVert],vertCW[endVert]);
            if((next - vertCW[startVert]).norm() < 0.005*bugStep){
                //std::cout << "hitpoint: " << getHit(bugXY,bugNext,t) << std::endl;
                wallPoint = vertCW[startVert];
                /*if(!followMode){
                    //qHit = vertCW[startVert];
                }*/
                currentOb = Ob;
                vertIdx = abs(startVert - numV);
                /*
                std::cout << "Hit corner, current obstacle 1st ccw vertex: " << currentOb.verticesCCW()[0] << std::endl;
                std::cout << "vertIdx: "<< vertIdx << std::endl;
                std::cout << "current obstacle vertIdx vertex: " << currentOb.verticesCCW()[vertIdx] << std::endl;
                std::cout << "endVert: "<< endVert <<std::endl;*/
                return true;
            }
            else if(evalTU(t,u)){
                //std::cout << "t: " << t << " u: " << u << " startVert: " << startVert<< " endVert: " << endVert << " hitpoint: " << getHit(bugXY,bugNext,t) << std::endl;
                //std::cout << " startVert Vertex: " << vertCW[startVert] << " endVert Vertex: " << vertCW[endVert] << std::endl;
                wallPoint = getHit(position,next,t);
                /*if(!followMode){
                    //qHit = getHit(bugXY,bugNext,t);
                    qHit = getNext(bugXY,problem.q_goal,getAlignDist(false));
                }*/
                currentOb = Ob;
                //std::cout << "current obstacle 1st ccw vertex: " << currentOb.verticesCCW()[0] << std::endl;
                if((startVert == numV && endVert == 0) || (startVert == 0 && endVert == numV)){
                    vertIdx = 0;
                }
                else{
                    startVert = abs(startVert - numV);
                    endVert = abs(endVert - numV);
                    (startVert > endVert) ? (vertIdx = startVert) : (vertIdx = endVert); 
                }
                /*std::cout << "bugXY: "<< position << "wallPoint: " << wallPoint << std::endl;
                std::cout << "vertIdx: "<< vertIdx << std::endl;
                std::cout << "current obstacle vertIdx vertex: " << currentOb.verticesCCW()[vertIdx] << std::endl;
                std::cout << "endVert: "<< endVert <<std::endl;*/
                return true;
            }
            return false;
        }
        bool checkMLine(Eigen::Vector2d init, Eigen::Vector2d goal){
            // Checks for intersection of next step and a closer M-line point
            double t = getT(bugXY,bugNext,init,goal);
            double u = getU(bugXY,bugNext,init,goal);
            if(evalTU(t,u)){
                Eigen::Vector2d testLeave = getHit(bugXY,bugNext,t);
                if(checkLeavePoint(testLeave,goal)){
                    // Set next step to be on M-line and to stop following the boundary
                    bugNext = testLeave;
                    followMode = false;
                    hit = true;
                    return true;
                }

            }
            return false;
        }
        double getQLScore(Eigen::Vector2d bugXY, Eigen::Vector2d goal){
            Eigen::Vector2d diff =  (goal - bugXY);
            return diff.norm();
        }
        bool checkLeavePoint(Eigen::Vector2d position, Eigen::Vector2d goal){
            // returns true if leavePoint changed
            double testScore = getQLScore(position,goal);
            if(testScore <= qLScore || qLScore == 0 ){
                qLeave = position;
                qLScore = testScore;
                return true;
            }
            else{
                return false;
            }
        }
        double getAlignDist(bool alignToCorner){
            if(alignToCorner){
                //assumes bug is bugStep away from the wall and in wall follow mode
                int endVert;
                (vertIdx == 0) ? (endVert = currentOb.verticesCCW().size() - 1) : endVert = vertIdx - 1;
                Eigen::Vector2d corner = currentOb.verticesCCW()[endVert];
                double norm = ((bugXY - corner).norm());
                double theta = acos(bugStep/norm);
                return norm*(sin(theta));                
            }
            else{
                Eigen::Vector2d pointD = currentOb.verticesCCW()[vertIdx];
                double theta1 = atan2((wallPoint(1) - pointD(1)),(wallPoint(0) - pointD(0)));
                double theta5 = atan2((wallPoint(1) - bugXY(1)),(wallPoint(0) - bugXY(0)));
                double theta2;
                //std::cout << "WALLPOINT: " << wallPoint << " pointD: " << pointD << " bugXY: " << bugXY << std::endl;
                if((wallPoint(1) - pointD(1)) == 0 && (wallPoint(0) - pointD(0)) == 0){
                    //moving towards corner
                    int endVert;
                    (vertIdx == 0) ? (endVert = currentOb.verticesCCW().size() - 1) : endVert = vertIdx - 1;
                    pointD = currentOb.verticesCCW()[endVert];
                    theta1 = atan2((wallPoint(1) - pointD(1)),(wallPoint(0) - pointD(0)));
                    theta2 = theta1 - theta5;
                }
                if((wallPoint(1) - pointD(1)) == 0){
                    //wall parallel to x-axis
                    theta2 = theta5;
                }
                else if((wallPoint(0) - pointD(0)) == 0){
                    //wall parallel to y-axis
                    theta2 = atan2((wallPoint(0) - bugXY(0)),(wallPoint(1) - bugXY(1)));
                }
                else{
                    theta2 = theta1 - theta5;
                }
                //std::cout << "theta1: " << theta1 << " theta5: " << theta5 << " theta2: " << theta2 << " bugStep/sin(theta2): " << bugStep/sin(theta2) << " norm: " << (wallPoint - bugXY).norm() << " output: " << (wallPoint - bugXY).norm() - bugStep/sin(theta2) << std::endl;
                return (wallPoint - bugXY).norm() - abs(bugStep/sin(theta2));
            }
            
        }
        bool checkExitPoint(Eigen::Vector2d position, Eigen::Vector2d next, Eigen::Vector2d exitPoint){
            //return (position - exitPoint).norm() + (exitPoint - next).norm() == (position - next).norm();
            double dx = next(0) - position(0);
            double dy = next(1) - position(1);

            if(dx == 0 && exitPoint(0) == position(0)){
                
                bool test1 = (exitPoint(1) >= position(1) && exitPoint(1) <= next(1));
                bool test2 = (exitPoint(1) <= position(1) && exitPoint(1) >= next(1));
                //std::cout << "pos: " << position << " exitPoint: " << exitPoint << " next: " << next << " tst1 "<<test1<<" tst2 " << test2 << std::endl;
                if(test1 || test2){
                    return true;
                }
            }
            else if(dy == 0 && exitPoint(1) == position(1)){
                bool test1 = (exitPoint(0) >= position(0) && exitPoint(0) <= next(0));
                bool test2 = (exitPoint(0) <= position(0) && exitPoint(0) >= next(0));
                //std::cout << "pos: " << position << " exitPoint: " << exitPoint << " next: " << next << " tst1 "<<test1<<" tst2 " << test2 << std::endl;
                if(test1 || test2){
                    return true;
                }
            }
            else{
                double u = (exitPoint(0) - position(0))/dx;
                double t = (exitPoint(1) - position(1))/dy;
                //std::cout << "pos: " << position << " exitPoint: " << exitPoint << " next: " << next << "u : "<< u << " t: " << t << std::endl;
                if(u == t && (u >= 0 && u <= 1.0)){
                    return true;
                }
            }
            return false;
        }
        amp::Path2D followBug1(const amp::Problem2D& problem, amp::Path2D path){
            int startVert = vertIdx;
            int endVert;
            (startVert == 0) ? (endVert = currentOb.verticesCCW().size() - 1) : endVert = startVert - 1;
            int stepsAway = 0;
            
            //move once more toward obstacle edge if able
            bugNext = getNext(bugXY,problem.q_goal,getAlignDist(false));
            stepCheck(bugXY,bugNext,problem);//hit set to false at start of stepCheck
            if(!hit){
                path.waypoints.push_back(bugNext);
                bugXY = bugNext;
                hit = true;
            }
            // Set distance from bug to goal at its probable max (initial hit point)
            checkLeavePoint(bugXY, problem.q_goal);
            //Switch to boundary following mode
            followMode = true;
            lastCornerDist =  (bugXY - problem.q_goal).norm();
            // set exit point to initial hit point
            Eigen::Vector2d exitPoint = bugXY;
            //std::cout << "Initial exitPoint: " << exitPoint << std::endl;
            //std::cout << "going from corner " << currentOb.verticesCCW()[startVert] << " to corner " << currentOb.verticesCCW()[endVert] << std::endl;
            do{
                    stepsAway++;
                    startVert = vertIdx;
                    (startVert == 0) ? (endVert = currentOb.verticesCCW().size() - 1) : endVert = startVert - 1;
                    /*if(stepsAway % 10000 == 0 || stepsAway == 1){
                        std::cout << " Headed to vertex " << currentOb.verticesCCW()[endVert] <<
                         " With start vert: "<< currentOb.verticesCCW()[startVert] << std::endl;
                    }*/
                    //step toward nearest clockwise vertex on obstacle
                    lastCornerDist = (bugXY - currentOb.verticesCCW()[endVert]).norm();
                    // bugNext = getNextFollow(bugXY,currentOb.verticesCCW()[startVert],currentOb.verticesCCW()[endVert],2*bugStep);
                    // stepCheck(bugXY,bugNext,problem); //hit set to false at start of stepCheck
                    // if(!hit)
                    // {
                    //     bugNext = getNextFollow(bugXY,currentOb.verticesCCW()[startVert],currentOb.verticesCCW()[endVert],bugStep);
                    //     path.waypoints.push_back(bugNext);
                    //     bugXY = bugNext;
                    //     if(abs((bugXY - currentOb.verticesCCW()[endVert]).norm() - lastCornerDist) > bugStep){
                    //         std::cout << " lastCornerDist " << lastCornerDist << " bugXY " << bugXY << " bugNext " << bugNext << std::endl;
                    //         bugNext = getNext(bugXY,currentOb.verticesCCW()[endVert],(bugXY - currentOb.verticesCCW()[endVert]).norm() - 0.05*bugStep);
                    //         stepCheck(bugXY,bugNext,problem);
                    //         if(!hit){
                    //             path.waypoints.push_back(bugNext);
                    //             bugXY = bugNext;
                    //         }
                    //         kill = true;
                    //         return path;
                    //     }
                    //     checkLeavePoint(bugXY, problem.q_goal);
                    // }
                        //std::cout << "Norm to end: " << (bugXY - currentOb.verticesCCW()[endVert]).norm() << std::endl;

                        //Check for corner (end of wall line segment)
                        if((bugXY - currentOb.verticesCCW()[endVert]).norm() <= 1.25*bugStep){
                            //std::cout << "cornering from " << bugXY << " around corner " << currentOb.verticesCCW()[endVert] << std::endl;
                            bool cornering = true;
                            bugNext = getNext(bugXY,currentOb.verticesCCW()[endVert],(bugXY - currentOb.verticesCCW()[endVert]).norm() - 0.1*bugStep);
                            stepCheck(bugXY,bugNext,problem);
                            if(!checkExitPoint(bugXY,bugNext,exitPoint) && !hit){
                                
                                path.waypoints.push_back(bugNext);
                                bugXY = bugNext;
                                //std::cout << "now at " << bugXY << std::endl;
                                Eigen::Vector2d tempXY = bugXY;
                                int endVertTemp;
                                Eigen::Vector2d corner = currentOb.verticesCCW()[endVert];
                                (endVert == 0) ? (endVertTemp = currentOb.verticesCCW().size() - 1) : endVertTemp = endVert - 1;
                                do{
                                    bugNext = getNextFollow(tempXY,corner,currentOb.verticesCCW()[endVertTemp],0.5*bugStep);
                                    stepCheck(tempXY,bugNext,problem);
                                    if(!hit){
                                        if(checkExitPoint(tempXY,bugNext,exitPoint) || (tempXY - exitPoint).norm() < 1.25*bugStep){
                                            /*std::cout << "Here Now retracing to leave point because a) " << (bugXY - exitPoint).norm() << " and b) " << stepsAway << std::endl;
                                            std::cout << "bugXY:  " << bugXY << " and exitPoint: "<< exitPoint << std::endl;
                                            std::cout << "tempXY: " << tempXY << std::endl;*/
                                            path.waypoints.push_back(tempXY);
                                            path.waypoints.push_back(exitPoint);
                                            bugXY = exitPoint;
                                            exitPoint = qLeave;
                                            followMode = false;
                                            cornering = false;
                                            vertIdx = endVert;
                                        }
                                        else{
                                            //std::cout << "final tempXY:  " << tempXY << " and bugNext: "<< bugNext << std::endl;
                                            path.waypoints.push_back(tempXY);
                                            path.waypoints.push_back(bugNext);
                                            bugXY = bugNext;
                                            //std::cout << " dist to exitpoint: " << (bugXY - exitPoint).norm() << std::endl;
                                            checkLeavePoint(bugXY, problem.q_goal);
                                            cornering = false;
                                            vertIdx = endVert;
                                            // Check if bug has hit original hit point
                                        }
                                    }
                                    else{
                                        if((currentOb.verticesCCW()[endVert] - corner).norm() > 0.00001){
                                            // hit new obstacle
                                            //std::cout << "new Obstacle! at " << bugXY << std::endl;
                                            //checkLeavePoint(bugXY, problem.q_goal);
                                            qLeave = bugXY;
                                            qLScore = (bugXY - problem.q_goal).norm();
                                            cornering = false;
                                        }
                                        else if((tempXY - currentOb.verticesCCW()[endVert]).norm() > bugStep){
                                            std::cout << "Missed corner at " << bugXY << " and tempXY " << tempXY << " bugNext: " << bugNext << std::endl;
                                            cornering = false;
                                            //kill = true;
                                            //return path;
                                        }
                                        tempXY = getNextFollow(tempXY,currentOb.verticesCCW()[startVert],currentOb.verticesCCW()[endVert],0.005);
                                        
                                    }
                                    
                                }while(cornering);
                                // update 'wall starts from vertex' to wall segment end vertex
                                
                            
                            }
                            else{
                                std::cout << "hit while going to corner at " << bugXY << " and corner " << currentOb.verticesCCW()[endVert] << " bugNext: " << bugNext << std::endl;
                            }
                            
                            /* old loop :))
                        {
                            int nextVert;
                            (endVert == 0) ? (nextVert = currentOb.verticesCCW().size() - 1) : nextVert = endVert - 1;
                            double phi = atan2(currentOb.verticesCCW()[nextVert](1) - currentOb.verticesCCW()[endVert](1), 
                            currentOb.verticesCCW()[nextVert](0) - currentOb.verticesCCW()[endVert](0)) - 
                            atan2(currentOb.verticesCCW()[vertIdx](1) - currentOb.verticesCCW()[endVert](1),
                            currentOb.verticesCCW()[vertIdx](0) - currentOb.verticesCCW()[endVert](0));
                            double theta0 = atan2(bugXY(1) - currentOb.verticesCCW()[endVert](1), bugXY(0) - currentOb.verticesCCW()[endVert](0)) - M_PI/2;
                            double thetaEnd = theta0 + M_PI/2;
                            //Check that taking the corner is safe
                            bugNext = getNextFollow(bugXY,currentOb.verticesCCW()[startVert],currentOb.verticesCCW()[endVert], getAlignDist(true));
                            stepCheck(bugXY,bugNext,problem);
                            if(!hit){
                                //path.waypoints.push_back(bugNext);
                                //bugXY = bugNext;
                                bugNext = getNextFollow(bugXY,currentOb.verticesCCW()[startVert],currentOb.verticesCCW()[endVert], 2*bugStep);
                                //std::cout << "startVert: " << currentOb.verticesCCW()[startVert] << " endVert: " << currentOb.verticesCCW()[endVert] <<
                                //" bugNext: " << bugNext << std::endl;
                                stepCheck(bugXY,bugNext,problem); //hit set to false at start of stepCheck
                                if(!hit){
                                    int endVertTemp;
                                    (startVert == 0) ? (endVert = currentOb.verticesCCW().size() - 1) : endVert = startVert - 1;
                                    (endVert == 0) ? (endVertTemp = currentOb.verticesCCW().size() - 1) : endVertTemp = endVert - 1;
                                    Eigen::Vector2d bugNextTemp = getNextFollow(bugNext,currentOb.verticesCCW()[endVert],currentOb.verticesCCW()[endVertTemp], 2*bugStep);
                                    stepCheck(bugNext,bugNextTemp,problem);
                                    //Take smooth corner
                                    if(!hit){
                                        std::cout << "Cornering here bugXY: "<< bugXY << " bugNext: " << bugNext << " endVert: " << currentOb.verticesCCW()[endVert] << std::endl;
                                        //std::cout << "starting cornering! at " << bugXY << std::endl;
                                        for(double  theta = thetaEnd; theta >= theta0; theta = theta - abs(M_PI - abs(phi))/15){
                                            if(!hit){
                                                if(!(theta == theta0 + abs(M_PI - abs(phi))/15)){
                                                    bugNext = getNextCorner(bugXY, currentOb.verticesCCW()[endVert], theta - abs(M_PI - abs(phi))/15);
                                                    stepCheck(bugXY,bugNext,problem);
                                                    if(!hit){
                                                        bugNext = getNextCorner(bugXY, currentOb.verticesCCW()[endVert], theta);
                                                    }
                                                }
                                                else{
                                                    bugNext = getNextCorner(bugXY, currentOb.verticesCCW()[endVert], theta);
                                                }
                                                path.waypoints.push_back(bugNext);
                                                bugXY = bugNext;
                                                checkLeavePoint(bugXY, problem.q_goal);
                                                if(!followMode && (bugXY - exitPoint).norm() < 1.05*bugStep && stepsAway > 2){
                                                    //std::cout << "hit leave point on corner, bugXY: "<< bugXY << std::endl;
                                                    hit = true; //Indicates we've hit the leave point
                                                }
                                            }
                                            else{
                                                // hit wall while cornering
                                                if(followMode){
                                                    /*std::cout << "Error: hit wall while cornering!" << std::endl;
                                                    path.waypoints.push_back(problem.q_goal);
                                                    return path;
                                                    hit = true;
                                                }
                                            }
                                        }
                                        // update 'wall starts from vertex' to wall segment end vertex
                                        checkLeavePoint(bugXY, problem.q_goal);
                                        vertIdx = endVert;
                                    }
                                    else{
                                        std::cout << "Error: hit wall before cornering! bugXY: "<< bugXY <<
                                        " bugNext: "<< bugNext << " bugNextTemp: "<< bugNextTemp  << " startVert: " << currentOb.verticesCCW()[startVert] << " endVert: " << currentOb.verticesCCW()[endVert] 
                                        << " entVertTemp: " << currentOb.verticesCCW()[endVertTemp] << " hitpoint: " << wallPoint << std::endl;
                                        bugNext = getNext(bugXY,wallPoint,getAlignDist(false));
                                        stepCheck(bugXY,bugNext,problem);//hit set to false at start of stepCheck
                                        if(!hit){
                                            std::cout<< "moving here " << bugNext << std::endl;
                                            path.waypoints.push_back(bugNext);
                                            bugXY = bugNext;
                                            hit = true;
                                        }
                                    }
                                }
                            }
                        }*/

                        }
                        else{
                        // Failsafe
                        
                        if(stepsAway > limit || abs(bugXY(1)) > 50 || abs(bugXY(0)) > 50){
                        //if(stepsAway > 20000){
                            kill = true;
                            std::cout << "Bug has left the working area!" << " bugxy: " << bugXY << " exitPoint: " << exitPoint << std::endl;
                            //path.waypoints.push_back(problem.q_init);
                            path.waypoints.push_back(problem.q_goal);
                            return path;
                        }
                    //}
                    bugNext = getNextFollow(bugXY,currentOb.verticesCCW()[startVert],currentOb.verticesCCW()[endVert],2*bugStep);
                    stepCheck(bugXY,bugNext,problem); //hit set to false at start of stepCheck
                    if(!hit)
                    {
                        bugNext = getNextFollow(bugXY,currentOb.verticesCCW()[startVert],currentOb.verticesCCW()[endVert],bugStep);
                        path.waypoints.push_back(bugNext);
                        bugXY = bugNext;
                        if(abs((bugXY - currentOb.verticesCCW()[endVert]).norm() - lastCornerDist) > bugStep){
                            std::cout << " lastCornerDist " << lastCornerDist << " bugXY " << bugXY << " bugNext " << bugNext << std::endl;
                            bugNext = getNext(bugXY,currentOb.verticesCCW()[endVert],(bugXY - currentOb.verticesCCW()[endVert]).norm() - 0.05*bugStep);
                            stepCheck(bugXY,bugNext,problem);
                            if(!hit){
                                path.waypoints.push_back(bugNext);
                                bugXY = bugNext;
                            }
                            kill = true;
                            return path;
                        }
                        checkLeavePoint(bugXY, problem.q_goal);
                    }
                    else{
                        //move once more toward obstacle wall if able
                        if(getAlignDist(false) > 0 ){
                            bugNext = getNext(bugXY,wallPoint,getAlignDist(false));
                            stepCheck(bugXY,bugNext,problem);//hit set to false at start of stepCheck
                            if(!hit){
                                //std::cout << "moving towards " << wallPoint << " by " << getAlignDist(false) << std::endl;
                                path.waypoints.push_back(bugNext);
                                bugXY = bugNext;
                                hit = true;
                            }
                        }
                        
                    }
                }
                    // Check if bug has hit original hit point
                    if(followMode && (bugXY - exitPoint).norm() < 1.5*bugStep && stepsAway > 2){
                        // Update point of loop exit to leave point
                        //std::cout << "Now retracing to leave point because a) " << (bugXY - exitPoint).norm() << " and b) " << stepsAway << std::endl;
                        //std::cout << "Leave point score:  " << qLScore << " and leavePoint: "<< qLeave << std::endl;
                        path.waypoints.push_back(exitPoint);
                        bugXY = exitPoint;
                        exitPoint = qLeave;
                        followMode = false;
                    }
                    else if(!followMode && (bugXY - problem.q_goal).norm() <= 1.05*qLScore && stepsAway > 2){
                        //std::cout << "DEBUG: Hit leave point! qLScore: " << qLScore << " norm: " << (bugXY - problem.q_goal).norm() << " bugXY: " << bugXY << " qLeave: " << qLeave << std::endl;
                    }
                    if(stepsAway % 10000 == 0){
                        std::cout << "stepsAway: " << stepsAway << " hit?: " << hit << " bugXY: " << bugXY << std::endl;
                    }
                    
                }while(((bugXY - exitPoint).norm() >= 1.5*bugStep || stepsAway < 2) && stepsAway < limit);
                //std::cout << "Now leaving object because a) " << (bugXY - exitPoint).norm() << " and b) " << stepsAway << std::endl;
                //std::cout << "Leave point score:  " << qLScore << " and leavePoint: "<< qLeave << std::endl;
                // reset leave point score
                qLScore = 0;
                if(stepsAway >= limit){
                    kill = true;
                }
                return path;
        }
        amp::Path2D followBug2(const amp::Problem2D& problem, amp::Path2D path){
            int startVert;
            int endVert;
            int stepsAway = 0;
            //move once more toward obstacle edge if able
            bugNext = getNext(bugXY,problem.q_goal,getAlignDist(false));
            stepCheck(bugXY,bugNext,problem);//hit set to false at start of stepCheck
            if(!hit){
                path.waypoints.push_back(bugNext);
                bugXY = bugNext;
                hit = true;
            }
            // Set distance from bug to goal at its probable max (initial hit point)
            checkLeavePoint(bugXY, problem.q_goal);
            //Switch to boundary following mode
            followMode = true;
            // set exit point to initial hit point
            Eigen::Vector2d exitPoint = bugXY;
            do{
                    startVert = vertIdx;
                    (startVert == 0) ? (endVert = currentOb.verticesCCW().size() - 1) : endVert = startVert - 1;
                    //std::cout << " Headed to vertex " << endVert << " With start vert: "<< startVert << std::endl;
                    //step toward nearest clockwise vertex on obstacle
                    bugNext = getNextFollow(bugXY,currentOb.verticesCCW()[startVert],currentOb.verticesCCW()[endVert],2*bugStep);
                    stepCheck(bugXY,bugNext,problem); //hit set to false at start of stepCheck
                    stepsAway++;
                    if(!hit)
                    {
                        bugNext = getNextFollow(bugXY,currentOb.verticesCCW()[startVert],currentOb.verticesCCW()[endVert],bugStep);
                        // Check that bug is sufficiently far away from initial hit point before checking for m-line intersections
                        if(followMode && (bugXY - exitPoint).norm() > 1.05*bugStep){
                            checkMLine(problem.q_init, problem.q_goal);
                        }
                        path.waypoints.push_back(bugNext);
                        bugXY = bugNext;
                        //stepsAway++;

                        //Check for corner (end of wall line segment)
                        if((bugXY - currentOb.verticesCCW()[endVert]).norm() <= 1.25*bugStep){
                            int nextVert;
                            (endVert == 0) ? (nextVert = currentOb.verticesCCW().size() - 1) : nextVert = endVert - 1;
                            double phi = atan2(currentOb.verticesCCW()[nextVert](1) - currentOb.verticesCCW()[endVert](1), 
                            currentOb.verticesCCW()[nextVert](0) - currentOb.verticesCCW()[endVert](0)) - 
                            atan2(currentOb.verticesCCW()[vertIdx](1) - currentOb.verticesCCW()[endVert](1),
                            currentOb.verticesCCW()[vertIdx](0) - currentOb.verticesCCW()[endVert](0));
                            double theta0 = atan2(bugXY(1) - currentOb.verticesCCW()[endVert](1), bugXY(0) - currentOb.verticesCCW()[endVert](0)) - M_PI/2;
                            double thetaEnd = theta0 + M_PI/2;
                            //Check that taking the corner is safe
                            bugNext = getNextFollow(bugXY,currentOb.verticesCCW()[startVert],currentOb.verticesCCW()[endVert], 2*bugStep);
                            stepCheck(bugXY,bugNext,problem); //hit set to false at start of stepCheck
                            int endVertTemp;
                            (endVert == 0) ? (endVertTemp = currentOb.verticesCCW().size() - 1) : endVertTemp = endVert - 1;
                            Eigen::Vector2d bugNextTemp = getNextFollow(bugNext,currentOb.verticesCCW()[endVert],currentOb.verticesCCW()[endVertTemp], 2*bugStep);
                            stepCheck(bugNext,bugNextTemp,problem);
                            //Take smooth corner
                            if(!hit){
                                for(double  theta = thetaEnd; theta >= theta0; theta = theta - abs(M_PI - abs(phi))/15){
                                    if(!hit){
                                        bugNext = getNextCorner(bugXY, currentOb.verticesCCW()[endVert], theta);
                                        stepCheck(bugXY,bugNext,problem);
                                        checkMLine(problem.q_init, problem.q_goal);
                                        path.waypoints.push_back(bugNext);
                                        bugXY = bugNext;
                                    }
                                    else{
                                        // hit wall while cornering
                                        if(followMode){
                                            std::cout << "Error: hit wall while cornering!" << std::endl;
                                            path.waypoints.push_back(problem.q_goal);
                                            return path;
                                        }
                                    }
                                }
                                // update 'wall starts from vertex' to wall segment end vertex
                                vertIdx = endVert;
                            }
                        }
                        // Failsafe
                        
                        if(abs(stepsAway > 50000 || bugXY(1)) > 100 || abs(bugXY(0)) > 100){
                        //if(stepsAway > 5000){
                            followMode = false;
                            std::cout << "Bug has left the working area!" << " bugxy: " << bugXY << std::endl;
                            path.waypoints.push_back(problem.q_goal);
                            return path;
                        }
                    }
                    else{
                        //move once more toward obstacle wall if able
                        bugNext = getNext(bugXY,wallPoint,getAlignDist(false));
                        stepCheck(bugXY,bugNext,problem);//hit set to false at start of stepCheck
                        if(!hit){
                            //checkMLine(problem.q_init, problem.q_goal);
                            path.waypoints.push_back(bugNext);
                            bugXY = bugNext;
                            hit = true;
                        }
                    }
                    // Check if bug has hit original hit point
                    /*if(followMode && (bugXY - exitPoint).norm() < 1.05*bugStep && stepsAway > 2){
                        // Update point of loop exit to leave point
                        std::cout << "Never found closer point on M-Line, solution not possible :((((" << std::endl;
                        std::cout << "Leave point score:  " << qLScore << " and leavePoint: "<< qLeave << std::endl;
                        path.waypoints.push_back(problem.q_goal);
                        followMode = false;
                    }*/
                }while(followMode);
                // reset leave point score
                //std::cout << "exiting follow mode at " << bugXY << std::endl;
                qLScore = 0;
                return path;
        }
        amp::Path2D followBug3(const amp::Problem2D& problem, amp::Path2D path){
            int startVert;
            int endVert;
            int stepsAway = 0;
            //move once more toward obstacle edge if able
            bugNext = getNext(bugXY,problem.q_goal,getAlignDist(false));
            stepCheck(bugXY,bugNext,problem);//hit set to false at start of stepCheck
            if(!hit){
                path.waypoints.push_back(bugNext);
                bugXY = bugNext;
                hit = true;
            }
            // Set distance from bug to goal at its probable max (initial hit point)
            checkLeavePoint(bugXY, problem.q_goal);
            //Switch to boundary following mode
            followMode = true;
            // set exit point to initial hit point
            Eigen::Vector2d exitPoint = bugXY;
            //std::cout << "exitPoint: " << exitPoint << std::endl;
            do{
                    startVert = vertIdx;
                    (startVert == 0) ? (endVert = currentOb.verticesCCW().size() - 1) : endVert = startVert - 1;
                    //std::cout << " Headed to vertex " << endVert << " With start vert: "<< startVert << std::endl;
                    //step toward nearest clockwise vertex on obstacle
                    bugNext = getNextFollow(bugXY,currentOb.verticesCCW()[startVert],currentOb.verticesCCW()[endVert],2*bugStep);
                    stepCheck(bugXY,bugNext,problem); //hit set to false at start of stepCheck
                    if(!hit)
                    {
                        bugNext = getNextFollow(bugXY,currentOb.verticesCCW()[startVert],currentOb.verticesCCW()[endVert],bugStep);
                        path.waypoints.push_back(bugNext);
                        bugXY = bugNext;
                        checkLeavePoint(bugXY, problem.q_goal);
                        stepsAway++;
                        //std::cout << "Norm to end: " << (bugXY - currentOb.verticesCCW()[endVert]).norm() << std::endl;

                        //Check for corner (end of wall line segment)
                        if((bugXY - currentOb.verticesCCW()[endVert]).norm() <= 1.25*bugStep){
                            
                            bool cornering = true;
                            Eigen::Vector2d tempXY = bugXY;
                            int endVertTemp;
                            do{
                                //try to step toward corner
                                bugNext = getNextFollow(tempXY,currentOb.verticesCCW()[startVert],currentOb.verticesCCW()[endVert],bugStep);
                                stepCheck(tempXY,bugNext,problem);
                                if(!hit){
                                    if(checkExitPoint(tempXY,bugNext,exitPoint)){
                                        std::cout << "Now retracing to leave point because a) " << (bugXY - exitPoint).norm() << " and b) " << stepsAway << std::endl;
                                        std::cout << "Leave point score:  " << qLScore << " and leavePoint: "<< qLeave << std::endl;
                                        path.waypoints.push_back(exitPoint);
                                        bugXY = exitPoint;
                                        exitPoint = qLeave;
                                        followMode = false;
                                        cornering = false;
                                    }
                                    else{
                                        //try turning around corner
                                        tempXY = bugNext;
                                        //std::cout << "tempXY: " << tempXY << std::endl;
                                        (endVert == 0) ? (endVertTemp = currentOb.verticesCCW().size() - 1) : endVertTemp = endVert - 1;
                                        bugNext = getNextFollow(tempXY,currentOb.verticesCCW()[endVert],currentOb.verticesCCW()[endVertTemp],2*bugStep);
                                        stepCheck(tempXY,bugNext,problem);
                                        std::cout << "tempXY:  " << tempXY << " and bugNext: "<< bugNext << std::endl;
                                        if(!hit){
                                            if(checkExitPoint(tempXY,bugNext,exitPoint)){
                                                std::cout << "Here Now retracing to leave point because a) " << (bugXY - exitPoint).norm() << " and b) " << stepsAway << std::endl;
                                                std::cout << "bugXY:  " << bugXY << " and exitPoint: "<< exitPoint << std::endl;
                                                std::cout << "tempXY: " << tempXY << std::endl;
                                                path.waypoints.push_back(tempXY);
                                                path.waypoints.push_back(exitPoint);
                                                bugXY = exitPoint;
                                                exitPoint = qLeave;
                                                followMode = false;
                                                cornering = false;
                                            }
                                            else{
                                                std::cout << "final tempXY:  " << tempXY << " and bugNext: "<< bugNext << std::endl;
                                                path.waypoints.push_back(tempXY);
                                                path.waypoints.push_back(bugNext);
                                                bugXY = bugNext;
                                                //std::cout << " dist to exitpoint: " << (bugXY - exitPoint).norm() << std::endl;
                                                checkLeavePoint(bugXY, problem.q_goal);
                                                cornering = false;
                                                // Check if bug has hit original hit point
                                            }
                                        }
                                        else{
                                            //path.waypoints.push_back(tempXY);
                                            //bugXY = tempXY;
                                            //checkLeavePoint(bugXY, problem.q_goal);
                                            //cornering = false;
                                        }
                                    }
                                }
                                
                            }while(cornering && stepsAway < limit);
                            // update 'wall starts from vertex' to wall segment end vertex
                            
                            checkLeavePoint(bugXY, problem.q_goal);
                            vertIdx = endVert;
                            std::cout << "bugXY: " << bugXY << " bugNext: "<< bugNext << " vertIdx " << vertIdx << std::endl;

                                
                            
                        }
                        // Failsafe
                        
                        if(abs(bugXY(1)) > 50 || abs(bugXY(0)) > 50 || stepsAway > limit){
                        //if(stepsAway > 20000){
                            std::cout << "Bug has left the working area!" << " bugxy: " << bugXY << std::endl;
                            path.waypoints.push_back(problem.q_goal);
                            return path;
                        }
                    }
                    else{
                        //move once more toward obstacle wall if able
                        bugNext = getNext(bugXY,wallPoint,getAlignDist(false));
                        stepCheck(bugXY,bugNext,problem);//hit set to false at start of stepCheck
                        if(!hit){
                            path.waypoints.push_back(bugNext);
                            bugXY = bugNext;
                            hit = true;
                        }
                    }
                    // Check if bug has hit original hit point
                    if(followMode && (bugXY - exitPoint).norm() < 1.5*bugStep && stepsAway > 2){
                        // Update point of loop exit to leave point
                        std::cout << "Now retracing to leave point because a) " << (bugXY - exitPoint).norm() << " and b) " << stepsAway << std::endl;
                        std::cout << "Leave point score:  " << qLScore << " and leavePoint: "<< qLeave << std::endl;
                        path.waypoints.push_back(exitPoint);
                        bugXY = exitPoint;
                        exitPoint = qLeave;
                        followMode = false;
                    }
                    else if(!followMode && (bugXY - problem.q_goal).norm() <= 1.05*qLScore && stepsAway > 2){
                        //std::cout << "DEBUG: Hit leave point! qLScore: " << qLScore << " norm: " << (bugXY - problem.q_goal).norm() << " bugXY: " << bugXY << " qLeave: " << qLeave << std::endl;
                    }
                }while((bugXY - exitPoint).norm() >= 1.5*bugStep || stepsAway < 2);
                std::cout << "Now leaving object because a) " << (bugXY - exitPoint).norm() << " and b) " << stepsAway << std::endl;
                std::cout << "Leave point score:  " << qLScore << " and leavePoint: "<< qLeave << std::endl;
                // reset leave point score
                qLScore = 0;
                return path;
        }
    private:
        // Add any member variables here...
        
};