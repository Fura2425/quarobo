/*
    DoF: 3
    LINK: 2
    JOINT: 3

   L1,  L2  -> link length.
   L1C, L2C -> link cos.
   L1S, L2S -> link sin.
*/

#include <math.h>
#include "kinematics.hpp"
#include "leg_parameter.hpp"

static LINK_PARAM link_parameter_3dof[LINK_NUM] = {0};
//static JOINT_RANGE joint_range[JOINT_NUM] = {0};

void initParam(void){
    getLinkParam3dof(link_parameter_3dof);
}

int forward_Kinematics3dof(VECTOR_3D *p, double *theta){

    double L1 = link_parameter_3dof[0].length;
    double L2 = link_parameter_3dof[1].length;

    double L1C = cos(theta[1]);
    double L2C = cos(theta[1] + theta[2]);
    double L1S = sin(theta[1]);
    double L2S = sin(theta[1] + theta[2]);

    p->x = L1 * L1C + L2 * L2C;
    p->y = L1 * L1S + L2 * L2S;

    return 0;
}

int inverse_Kinematics3dof(VECTOR_3D *p, double *theta){

/* ???
z = -sqrt(z^2+y^2)
c = sqrt(x^2+z^2)
D1 = atan2(x,z)
D2 = acos((c^2 + L1^2 - L2^2) / (2cL1))

a = -atan2(y,z) + PI
b = D1 + D2
y = acos((L1^2 + L2^2 - c^2) / (2*L1*L2)) - PI
*/

}
