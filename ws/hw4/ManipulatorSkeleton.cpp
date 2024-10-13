#include "ManipulatorSkeleton.h"
#include<Eigen/Geometry>


MyManipulator2D::MyManipulator2D()
    : LinkManipulator2D({1.0, 1.0}) // Default to a 2-link with all links of 1.0 length
{}

MyManipulator2D::MyManipulator2D(const std::vector<double>& link_lengths) : LinkManipulator2D(link_lengths){}

    

Eigen::Vector2d MyManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
    std::cout << "HERE" << "\n";
    Eigen::Vector2d result; result << 0, 0;
    double theta = 0;
    for (int i = 0; i < joint_index; i++){
        theta+= state[i];
        result[0] += getLinkLengths()[i]*cos(theta);
        result[1] += getLinkLengths()[i]*sin(theta);
    }
    return result;
}

// Override this method for implementing inverse kinematics
amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    // Implement inverse kinematics here

    std::cout << nLinks();

    amp::ManipulatorState joint_angles(nLinks());
    joint_angles.setZero();
    //double reach = reach();
    double dist = sqrt(pow(end_effector_location[1], 2) + pow(end_effector_location[0], 2));
    // if (reach == dist){
    //     joint_angles.setZero();
    //     return joint_angles;
    // }
    
    double x = end_effector_location[0];
    double y = end_effector_location[1];
    std::cout << "HERE1" << "\n";

    // If you have different implementations for 2/3/n link manipulators, you can separate them here
    if (nLinks() == 2) {
        
        std::vector<amp::ManipulatorState> angle_options;
        double a1 =  getLinkLengths()[0];
        double a2 =  getLinkLengths()[1];
        double cos2 = (1 / (2*a1*a2)) * ((pow(x, 2) + pow(y, 2))-(pow(a1, 2) + pow(a2, 2)));

        double theta2 = acos(cos2);
        // theta2options[0] = acos(cos2);
        // theta2options[1] = -acos(cos2);
        double cos1 = (1/(pow(x, 2) + pow(y, 2))) * (x*(a1 + a2*cos2) + y*(a2*sqrt(1-pow(cos2, 2))));
        double theta1 = acos(cos1);
        
        amp::ManipulatorState optiona(2);
        optiona << theta1, theta2;
        angle_options.push_back(optiona);
        amp::ManipulatorState optionb(2); optionb << -1*theta1, theta2;
        angle_options.push_back(optionb);
        amp::ManipulatorState optionc(2); optionc << -1*theta1, -1*theta2;
        angle_options.push_back(optionc);
        amp::ManipulatorState optiond(2); optiond << theta1, -1*theta2;
        angle_options.push_back(optiond);
        std::cout << "angle option size" << angle_options.size() << "\n";
        for (int i = 0; i < angle_options.size(); i++){
            Eigen::Vector2d endloc = getJointLocation(angle_options[i], 2);
            std::cout<< "my guess : " << endloc[0] << ", " << endloc[1] << "\n";
            std::cout<< "given x and y: " << x << ", " << y << "\n";
            if ((abs(endloc[0] - x) <= .1) && (abs(endloc[1] - y) <= .1)){
                std::cout << "here " << "\n";
                joint_angles << angle_options[i][0], angle_options[i][1];
                return joint_angles;
            }
        }
        return joint_angles;
    } else if (nLinks() == 3) {
        for (double i = 0; i < 2*M_PI; i = i + 0.01){
            //amp::ManipulatorState joint_angles(3);
            double theta1 = i;
            double a1 =  getLinkLengths()[0];
            double a2 =  getLinkLengths()[1];
            double a3 =  getLinkLengths()[2];
            double cos3 = (1 / (2*a2*a3)) * ((pow((x - a1*cos(theta1)), 2) + pow((y - a1*sin(theta1)), 2))-(pow(a2, 2) + pow(a3, 2)));
            double theta3 = acos(cos3);
            double cos2 = (1/(pow(x - a1*cos(theta1), 2) + pow((y - a1*sin(theta1)), 2))) * (x - a1*cos(theta1)*(a2 + a3*cos3) + (y - a1*sin(theta1))*(a3*sqrt(1-pow(cos3, 2))));
            double theta2 = acos(cos2);
            //CHECK FORWARD KINEMATICS SEE IF IT WORKS AND IF NOT THEN INCREMENT ANGLE AND TRY AGAIN
            std::vector<amp::ManipulatorState> angle_options;
            amp::ManipulatorState optiona(3);
            optiona << theta1, theta2, theta3;
            angle_options.push_back(optiona);
            amp::ManipulatorState optionb(3); optionb << theta1, -1*theta2, theta3;
            angle_options.push_back(optionb);
            amp::ManipulatorState optionc(3); optionc << theta1, -1*theta2, -1*theta3;
            angle_options.push_back(optionc);
            amp::ManipulatorState optiond(3); optiond << theta1, theta2, -1*theta3;
            angle_options.push_back(optiond);
            std::cout << "angle option size" << angle_options.size() << "\n";
            for (int j = 0; j < angle_options.size(); j++){
                Eigen::Vector2d endloc = getJointLocation(angle_options[j], 3);
                std::cout<< "my guess : " << endloc[0] << ", " << endloc[1] << "\n";
                std::cout<< "given x and y: " << x << ", " << y << "\n";
                if ((abs(endloc[0] - x) <= .005) && (abs(endloc[1] - y) <= .005)){
                    std::cout << "here " << "\n";
                    joint_angles << theta1, angle_options[j][1], angle_options[j][2];
                   

                    return joint_angles;
                }
            }


            joint_angles << theta1, theta2, theta3;
            
        }
        std::cout<< "FAILED TO FIND ANGLE " << "\n";
        return joint_angles;
    } else {
        
        return joint_angles;
    }
    //amp::ManipulatorState joint_angles(nLinks());
    return joint_angles;
}
