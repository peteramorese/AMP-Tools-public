/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Justin Kottinger */

#include <boost/geometry.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <set>

namespace ob = ompl::base;

class isStateValid_2D : public ob::StateValidityChecker
{
public:
    isStateValid_2D(const ob::SpaceInformationPtr &si, const World *w, const Agent *a) : ob::StateValidityChecker(si) {
        si_ = si.get();
        w_ = w;
        a_ = a;
    }

    bool isValid(const ob::State *state) const override {
        if (!si_->satisfiesBounds(state))
            return false;

        // Get xy state and theta state (in rad [0, 2*pi]) from state object
        auto compState = state->as<ob::CompoundStateSpace::StateType>();
        auto xyState = compState->as<ob::RealVectorStateSpace::StateType>(0);
        const double cx = xyState->values[0];
        const double cy = xyState->values[1];
        const double theta = compState->as<ob::SO2StateSpace::StateType>(1)->value;

        // Get params from car object
        const double half_l = a_->getShape()[0] / 2;
        const double half_w = a_->getShape()[1] / 2;

        double cos_theta = std::cos(theta);
        double sin_theta = std::sin(theta);

        // Center point
        double ref_x = cx + half_l * cos_theta;
        double ref_y = cy + half_l * sin_theta;

        // Precompute repeated terms
        double l_cos = half_l * cos_theta;
        double l_sin = half_l * sin_theta;
        double w_cos = half_w * cos_theta;
        double w_sin = half_w * sin_theta;

        // turn (x,y, theta), width, length to a polygon object
        // see https://stackoverflow.com/questions/41898990/find-corners-of-a-rotated-rectangle-given-its-center-point-and-rotation
        // VERTICES:
        std::string top_right = std::to_string(ref_x + l_cos - w_sin) + " " + std::to_string(ref_y + l_sin + w_cos);
        std::string top_left = std::to_string(ref_x - l_cos - w_sin) + " " + std::to_string(ref_y - l_sin + w_cos);
        std::string bottom_left = std::to_string(ref_x - l_cos + w_sin) + " " + std::to_string(ref_y - l_sin - w_cos);
        std::string bottom_right = std::to_string(ref_x + l_cos + w_sin) + " " + std::to_string(ref_y + l_sin - w_cos);

        // convert to string for easy initializataion
        std::string points = "POLYGON((" + bottom_left + "," + bottom_right + "," + top_right + "," + top_left + "," + bottom_left + "))";
        polygon agent;
        boost::geometry::read_wkt(points, agent);

        // check agent is disjoint from all obstacles
        for (Obstacle o : w_->getObstacles())
            if (!boost::geometry::disjoint(agent, o.poly_))
                return false;
        return true;
    }

private:
    const ob::SpaceInformation *si_;
    const World *w_;
    const Agent *a_;
};
