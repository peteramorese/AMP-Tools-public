#pragma once

#include "AMPCore.h"
#include "hw/HW5.h"

using std::vector, std::string, Eigen::Vector2d, std::cout;

class MyGDAlgorithm : public amp::GDAlgorithm {
	public:
		// MyGDAlgorithm();
		void init(const amp::Problem2D& problem);
		void takeStep();
		Vector2d getGradient();
		Vector2d findAttractive();
		Vector2d findRepulsive();
		Vector2d findRepulsive(const amp::Obstacle2D& obstacle);
		virtual amp::Path2D plan(const amp::Problem2D& problem) override;
	private:
		Vector2d position;
		Vector2d start;
		Vector2d goal;
		double stepSize;
    	amp::Path2D path;
		vector<amp::Obstacle2D> obstacles;
		bool goalReached;

		double zetta;
		double dStar;
		double eta;
		double qStar;
		double E;

	// Add any member variables here...
};