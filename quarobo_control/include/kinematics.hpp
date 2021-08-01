/*
    DoF: 3
    LINK: 2
    JOINT: 3
*/
#ifndef _KINEMATICS_HPP_
#define _KINEMATICS_HPP_

#include "matrix.hpp"

#define LINK_NUM  2
#define JOINT_NUM 3

void initParam(void);

int forward_Kinematics3dof(VECTOR_3D *, double *);
int inverse_Kinematics3dof(VECTOR_3D *, double *);

#endif
