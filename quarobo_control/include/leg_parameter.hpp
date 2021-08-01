#ifndef _LEG_PARAMETER_
#define _LEG_PARAMETER_

#define LINK_NUM 2

typedef struct{
    double length;
    double intercept_x;
    double intercept_y;
    double intercept_z;
} LINK_PARAM;

//typedef struct{
//    double min;
//    double max;
//} JOINT_RANGE;

void getLinkParam3dof(LINK_PARAM *);

#endif
