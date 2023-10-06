#include "MyLinkManipulator.h"
#include <Eigen/Dense>

using std::vector, std::string, std::cout, Eigen::Vector2d;
using namespace amp;

Eigen::Matrix<double, 3, 3> getTMatrix(double length, double theta) {
    Eigen::Matrix<double, 3, 3> T; // 3x2 matrix
    // Fill the matrices A and B with values (example values)
    T << cos(theta), -sin(theta), length,
         sin(theta), cos(theta), 0,
         0, 0, 1;
    // cout << "\nAngle: " << theta << std::endl;
    // cout << "Matrix T:" << std::endl << T << std::endl;
    return T;
}

MyLinkManipulator::MyLinkManipulator() : LinkManipulator2D() {}

MyLinkManipulator::MyLinkManipulator(const std::vector<double>& link_lengths) 
    : LinkManipulator2D(link_lengths) {}


Vector2d MyLinkManipulator::getJointLocation(const ManipulatorState& state, uint32_t joint_index) const {
    vector<double> linkLengths = getLinkLengths();
    Vector2d jointLocation;
    if (joint_index == 0) {
        jointLocation = getBaseLocation();
    } else {
        Eigen::Matrix<double, 3, 1> location;
        Eigen::Matrix<double, 3, 3> translationT;
        translationT << 1, 0, linkLengths[joint_index-1],
            0, 1, 0,
            0, 0, 1;
        location << 0,
                0,
                1;
        location = translationT * location; 
        for (int i = joint_index; i > 0; --i) {
            double length;
            if (i == 1) {
                length = 0;
            }else{
                length = linkLengths[i-2];
            };
            location = getTMatrix(length, state[i-1]) * location;
            // cout << "Vector result:" << std::endl << location << std::endl;
        }
        jointLocation = {location.coeff(0, 0), location.coeff(1, 0)};
    }
    // cout << "Joint " << joint_index << " at: ("<< jointLocation(0) << ", " << jointLocation(1) << ")\n";
    return jointLocation;
};

ManipulatorState MyLinkManipulator::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    ManipulatorState state;
    std::vector<double> links = getLinkLengths();
    for (int i = 0; i < 12; ++i) {
        double theta3 = 2 * M_PI / 12 * i;
        double x = links[2]*cos(theta3) + end_effector_location(0);
        double y = links[2]*sin(theta3) + end_effector_location(1);
        // cout << "Point: ( " << x << ", " << y << " )\n";
        double cosTheta2 = ((pow(x, 2) + pow(y, 2)) - (pow(links[0], 2) + pow(links[1], 2)))/(2*links[0]*links[1]);
        double cosTheta1 = (x*(links[0] + links[1]*cosTheta2)+y*links[1]*sqrt(1-pow(cosTheta2, 2)))/(pow(x, 2) + pow(y, 2));
        state = {acos(cosTheta1), acos(cosTheta2), theta3 - acos(cosTheta2) - acos(cosTheta1) - M_PI};
        bool pass = true;
        for (auto element : state) {            
            if (std::isnan(element)) pass = false;
        }
        if (pass) break;
    }
    return state;
};
