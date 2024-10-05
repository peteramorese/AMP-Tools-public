#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"

class MyGDAlgorithm : public amp::GDAlgorithm {
	public:
		// Consider defining a class constructor to easily tune parameters, for example: 
		MyGDAlgorithm(double d_star, double zetta, double Q_star, double eta) :
			d_star(d_star),
			zetta(zetta),
			Q_star(Q_star),
			eta(eta) {}

		// Override this method to solve a given problem.
		virtual amp::Path2D plan(const amp::Problem2D& problem) override;
	private:
		double d_star, zetta, Q_star, eta;
		// Add additional member variables here...
};

class MyPotentialFunction : public amp::PotentialFunction2D {
    public:
		MyPotentialFunction(amp::Problem2D& prob) : problem(prob){
			//problem = prob;
		}
		// Returns the potential function value (height) for a given 2D point. 
        virtual double operator()(const Eigen::Vector2d& q) const override {
			
			double d_star = 5; //CHANGE ALL OF THESE TO PARAMETERS
			double zetta = 1;
			double Q_star = 1;
			double eta = 0.5;

			//ATTRACTIVE FORCE
			double att = 0;
			double euclid = euclidian(q, problem.q_goal);
			if (euclid <= d_star){
				att = 0.5 * zetta * pow(euclid, 2);
			}
			else{
				att = d_star * zetta * euclid - 0.5 * zetta * pow(d_star, 2);
			}

			//REPULSIVE FORCE
			double rep = 0;
			std::vector<amp::Obstacle2D> obstacles = problem.obstacles;
			for (amp::Obstacle2D obs : obstacles){
				//std::cout<<"checking obs" << "\n";
				double di = dist_to_obs(q, obs);
				std::cout<< di << "\n";
				if (di <= Q_star){
					//std::cout<<"adding to rep" << "\n";
					rep += 0.5 * eta * pow(((1 / di)-(1/Q_star)), 2);
				}
			}
            return att + rep;
        }

		double euclidian(const Eigen::Vector2d& q, const Eigen::Vector2d& p) const{
			return sqrt(pow((q[0]-p[0]), 2) + pow(q[1]-p[1],2));
			// return 1;
		}

		double dist_to_obs(const Eigen::Vector2d& q, const amp::Obstacle2D obs) const{
			//TODO: might have to edit this so that it takes into accounts all points on the obstacle instead of just the vertices, but since it's convex I think it's mostly fine
			std::vector<Eigen::Vector2d> verts = obs.verticesCCW();
			double dist = euclidian(q, verts[0]);
			for (Eigen::Vector2d vert : verts){
				if (euclidian(q, vert) < dist){ dist=euclidian(q, vert); } 
			}
			return dist;
		}
	private: 
		amp::Problem2D& problem;
};


