#pragma once

#include "AMPCore.h"
#include "hw/HW5.h"
#include "HelpfulClass.h"

using std::vector, std::string, Eigen::Vector2d, std::cout;

class MyGDAlgorithm : public amp::GDAlgorithm {
	public:
		MyGDAlgorithm(double zetta, double dStar, double eta, double qStar, double E) :
			zetta(zetta),
			dStar(dStar),
			eta(eta),
			qStar(qStar),
			E(E),
			step(0) {}
		void init(const amp::Problem2D& problem);
		void takeStep();
		Vector2d getGradient();
		Vector2d findAttractive();
		Vector2d findRepulsive(const vector<vector<Edge>>& polyRegions);
		Vector2d findClosestPoint(const vector<Edge>& region);
		vector<Edge> findRegion(const vector<vector<Edge>>& polyRegions);

		virtual amp::Path2D plan(const amp::Problem2D& problem) override;
	private:
		Vector2d position;
		Vector2d start;
		Vector2d goal;
		double stepSize;
		bool goalReached;
		int step;    	
		amp::Path2D path;
		vector<amp::Obstacle2D> obstacles;
		vector<vector<vector<Edge>>> regions;

		double zetta;
		double dStar;
		double eta;
		double qStar;
		double E;

	// Add any member variables here...
};