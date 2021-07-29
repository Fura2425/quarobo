/*
   Process the value of JointState and publish it to Arduino.
   */
#include <math.h>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Int32MultiArray.h"


// prototype
int value_map(float x, float in_min, float in_max, int out_min, int out_max);
double value_truncation(double x, double y);


std_msgs::Int32MultiArray joint_angle;

void jointStates_processing_cb(const sensor_msgs::JointState& joint_states){
    // FL
    joint_angle.data[0] = value_map(value_truncation(joint_states.position[2], 100), -2.61, 2.61, 2480, 560);
    joint_angle.data[1] = value_map(value_truncation(joint_states.position[1], 100), -2.61, 2.61, 2480, 560);
    joint_angle.data[2] = value_map(value_truncation(joint_states.position[0], 100), -2.61, 2.61, 2480, 560);

    // FR
    joint_angle.data[3] = value_map(value_truncation(joint_states.position[5], 100), -2.61, 2.61, 2480, 560);
    joint_angle.data[4] = value_map(value_truncation(joint_states.position[4], 100), -2.61, 2.61, 560, 2480);
    joint_angle.data[5] = value_map(value_truncation(joint_states.position[3], 100), -2.61, 2.61, 2480, 560);

    // RL
    joint_angle.data[6] = value_map(value_truncation(joint_states.position[8], 100), -2.61, 2.61, 560, 2480);
    joint_angle.data[7] = value_map(value_truncation(joint_states.position[7], 100), -2.61, 2.61, 2480, 560);
    joint_angle.data[8] = value_map(value_truncation(joint_states.position[6], 100), -2.61, 2.61, 560, 2480);

    // RR
    joint_angle.data[9] = value_map(value_truncation(joint_states.position[11], 100),   -2.61, 2.61, 560, 2480);
    joint_angle.data[10] = value_map(value_truncation(joint_states.position[10], 100), -2.61, 2.61, 560, 2480);
    joint_angle.data[11] = value_map(value_truncation(joint_states.position[9], 100), -2.61, 2.61, 2480, 560);

    ROS_INFO("--Joint States-------");
    ROS_INFO("FLS_joint  pulse: %d, angle: %lf", joint_angle.data[0], value_truncation(joint_states.position[2], 100));
    ROS_INFO("FLL_joint  pulse: %d, angle: %lf", joint_angle.data[1], value_truncation(joint_states.position[1], 100));
    ROS_INFO("FLF_joint  pulse: %d, angle: %lf", joint_angle.data[2], value_truncation(joint_states.position[0], 100));
    ROS_INFO("FRS_joint  pulse: %d, angle: %lf", joint_angle.data[3], value_truncation(joint_states.position[5], 100));
    ROS_INFO("FRL_joint  pulse: %d, angle: %lf", joint_angle.data[4], value_truncation(joint_states.position[4], 100));
    ROS_INFO("FRF_joint  pulse: %d, angle: %lf", joint_angle.data[5], value_truncation(joint_states.position[3], 100));
    ROS_INFO("RLS_joint  pulse: %d, angle: %lf", joint_angle.data[6], value_truncation(joint_states.position[8], 100));
    ROS_INFO("RLL_joint  pulse: %d, angle: %lf", joint_angle.data[7], value_truncation(joint_states.position[7], 100));
    ROS_INFO("RLF_joint  pulse: %d, angle: %lf", joint_angle.data[8], value_truncation(joint_states.position[6], 100));
    ROS_INFO("RRS_joint  pulse: %d, angle: %lf", joint_angle.data[9], value_truncation(joint_states.position[11], 100));
    ROS_INFO("RRL_joint  pulse: %d, angle: %lf", joint_angle.data[10], value_truncation(joint_states.position[10], 100));
    ROS_INFO("RRF_joint  pulse: %d, angle: %lf", joint_angle.data[11], value_truncation(joint_states.position[9], 100));
    ROS_INFO("--------------------\n");
}

int main(int argc, char **argv){

    ros::init(argc, argv, "jointStates_processing");
    ros::NodeHandle nh;

    joint_angle.data.resize(12);

    for(int i=0; i<joint_angle.data.size(); i++){
        joint_angle.data[i] = 90;
    }

    ros::Subscriber quarobo_SointState_sub = nh.subscribe("/quarobo/joint_states", 256, jointStates_processing_cb);

    ros::Publisher jointState_publish = nh.advertise<std_msgs::Int32MultiArray>("joint_angle_pulse", 120);
    ros::Rate loop_rate(10);

    while(ros::ok()){
        jointState_publish.publish(joint_angle);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

int value_map(float x, float in_min, float in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double value_truncation(double x, double y){
    int x1 = x * y;
    x = (double)x1 / y;
    return x;
}
