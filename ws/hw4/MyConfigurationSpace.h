#pragma once

#include "AMPCore.h"
#include "hw/HW4.h"
#include <Eigen/LU>

class MyConfigurationSpace: public amp::ConfigurationSpace2D{
    public:
       virtual bool inCollision(double x0, double x1) const{
            return true;
       }
    private:

};