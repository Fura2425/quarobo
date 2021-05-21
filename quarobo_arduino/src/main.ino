#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <Servo.h>

ros::NodeHandle nh;

// Front Left        // Front Right
int FLS_angle = 90;  int FRS_angle = 90;
int FLL_angle = 90;  int FRL_angle = 90;
int FLF_angle = 90;  int FRF_angle = 90;

// Rear Left         // Rear Right
int RLS_angle = 90;  int RRS_angle = 90;
int RLL_angle = 90;  int RRL_angle = 90;
int RLF_angle = 90;  int RRF_angle = 90;

Servo FRS_sv, FRL_sv, FRF_sv,
      FLS_sv, FLL_sv, FLF_sv,
      RRS_sv, RRL_sv, RRF_sv,
      RLS_sv, RLL_sv, RLF_sv;

void servo_cb(const std_msgs::Int32MultiArray& joint_array){
    FRS_angle = joint_array.data[0];
    FRL_angle = joint_array.data[1];
    FRF_angle = joint_array.data[2];

    FLS_angle = joint_array.data[3];
    FLL_angle = joint_array.data[4];
    FLF_angle = joint_array.data[5];

    RRS_angle = joint_array.data[6];
    RRL_angle = joint_array.data[7];
    RRF_angle = joint_array.data[8];

    RLS_angle = joint_array.data[9];
    RLL_angle = joint_array.data[10];
    RLF_angle = joint_array.data[11];
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("processed_jointState", servo_cb);

void setup(){
    pinMode(13, OUTPUT);
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(sub);

    FRS_sv.attach(39);
    FRL_sv.attach(40);
    FRF_sv.attach(41);

    FLS_sv.attach(42);
    FLL_sv.attach(43);
    FLF_sv.attach(44);

    RRS_sv.attach(45);
    RRL_sv.attach(46);
    RRF_sv.attach(47);

    RLS_sv.attach(48);
    RLL_sv.attach(49);
    RLF_sv.attach(50);
}

void loop(){
    FLS_sv.write(FLS_angle);  FRS_sv.write(FRS_angle);
    FLL_sv.write(FLL_angle);  FRL_sv.write(FRL_angle);
    FLF_sv.write(FLF_angle);  FRF_sv.write(FRF_angle);

    RLS_sv.write(RLS_angle);  RRS_sv.write(RRS_angle);
    RLL_sv.write(RLL_angle);  RRL_sv.write(RRL_angle);
    RLF_sv.write(RLF_angle);  RRF_sv.write(RRF_angle);

    nh.spinOnce();
    delay(1);
}
