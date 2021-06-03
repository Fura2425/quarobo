/*
   Process the value of JointState and publish it to Arduino.
   */
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Int32MultiArray.h"

int value_map(float x, float in_min, float in_max, int out_min, int out_max);

std_msgs::Int32MultiArray joint_array;

void joint_state_cb(const sensor_msgs::JointState& joint_states){
    joint_array.data[0] = value_map(joint_states.position[0], -2.61, 2.61, 2480, 560);
    joint_array.data[1] = value_map(joint_states.position[1], -2.61, 2.61, 560, 2480);
    joint_array.data[2] = value_map(joint_states.position[2], -2.61, 2.61, 2480, 560);

    joint_array.data[3] = value_map(joint_states.position[3], -2.61, 2.61, 2480, 560);
    joint_array.data[4] = value_map(joint_states.position[4], -2.61, 2.61, 2480, 560);
    joint_array.data[5] = value_map(joint_states.position[5], -2.61, 2.61, 560, 2480);

    joint_array.data[6] = value_map(joint_states.position[6], -2.61, 2.61, 560, 2480);
    joint_array.data[7] = value_map(joint_states.position[7], -2.61, 2.61, 560, 2480);
    joint_array.data[8] = value_map(joint_states.position[8], -2.61, 2.61, 2480, 560);

    joint_array.data[9] = value_map(joint_states.position[9], -2.61, 2.61, 560, 2480);
    joint_array.data[10] = value_map(joint_states.position[10], -2.61, 2.61, 2480, 560);
    joint_array.data[11] = value_map(joint_states.position[11], -2.61, 2.61, 560, 2480);

    ROS_INFO("--Joint Angle-------");
    ROS_INFO("FRS_joint  angle: %d\n", joint_array.data[0]);
    ROS_INFO("FRL_joint  angle: %d\n", joint_array.data[1]);
    ROS_INFO("FRF_joint  angle: %d\n", joint_array.data[2]);
    ROS_INFO("FLS_joint  angle: %d\n", joint_array.data[3]);
    ROS_INFO("FLL_joint  angle: %d\n", joint_array.data[4]);
    ROS_INFO("FLF_joint  angle: %d\n", joint_array.data[5]);
    ROS_INFO("RRS_joint  angle: %d\n", joint_array.data[6]);
    ROS_INFO("RRL_joint  angle: %d\n", joint_array.data[7]);
    ROS_INFO("RRF_joint  angle: %d\n", joint_array.data[8]);
    ROS_INFO("RLS_joint  angle: %d\n", joint_array.data[9]);
    ROS_INFO("RLL_joint  angle: %d\n", joint_array.data[10]);
    ROS_INFO("RLF_joint  angle: %d\n", joint_array.data[11]);
    ROS_INFO("--------------------\n");
}

int main(int argc, char **argv){

    ros::init(argc, argv, "jointState_processing");
    ros::NodeHandle nh;

    joint_array.data.resize(12);

    for(int i=0; i<joint_array.data.size(); i++){
        joint_array.data[i] = 90;
    }

    ros::Subscriber jointState_subscribe = nh.subscribe("/quarobo/joint_states", 256, joint_state_cb);

    ros::Publisher jointState_publish = nh.advertise<std_msgs::Int32MultiArray>("processed_jointState", 120);
    ros::Rate loop_rate(10);

    while(ros::ok()){
        jointState_publish.publish(joint_array);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

int value_map(float x, float in_min, float in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
