/*
    DoF: 3
    LINK: 2
    JOINT: 3
*/
#include <iostream>
#include "../include/leg_parameter.hpp"
#include "../include/kinematics.hpp"

using namespace std;

int main(int argc, char **argv){

    double angle[JOINT_NUM] = {0.00, 0.60, -1.00};
    VECTOR_3D pose = {0};

    initParam();

    forward_Kinematics3dof(&pose, angle);

    cout << "Forward Kinematics result: " << pose.x << ", " << pose.y << endl;

    return 0;
}
