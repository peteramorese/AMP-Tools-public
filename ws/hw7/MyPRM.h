#pragma once
#include "AMPCore.h"
#include "hw/HW7.h"
namespace amp {
    class MyPRM : public amp::PRM2D {
        public:
            virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
    };


}


