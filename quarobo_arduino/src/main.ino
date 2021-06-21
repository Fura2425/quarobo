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
int FLS_angle = 1520;  int FRS_angle = 1520;
int FLL_angle = 1520;  int FRL_angle = 1520;
int FLF_angle = 1520;  int FRF_angle = 1520;

// Rear Left         // Rear Right
int RLS_angle = 1520;  int RRS_angle = 1520;
int RLL_angle = 1520;  int RRL_angle = 1520;
int RLF_angle = 1520;  int RRF_angle = 1520;

Servo FLS_sv, FLL_sv, FLF_sv,
      FRS_sv, FRL_sv, FRF_sv,
      RLS_sv, RLL_sv, RLF_sv,
      RRS_sv, RRL_sv, RRF_sv;

void servo_cb(const std_msgs::Int32MultiArray& joint_array){
    FLS_angle = joint_array.data[0];
    FLL_angle = joint_array.data[1];
    FLF_angle = joint_array.data[2];

    FRS_angle = joint_array.data[3];
    FRL_angle = joint_array.data[4];
    FRF_angle = joint_array.data[5];

    RLS_angle = joint_array.data[6];
    RLL_angle = joint_array.data[7];
    RLF_angle = joint_array.data[8];

    RRS_angle = joint_array.data[9];
    RRL_angle = joint_array.data[10];
    RRF_angle = joint_array.data[11];
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("joint_angle_pulse", servo_cb);

void setup(){
    pinMode(13, OUTPUT);
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(sub);

    FLS_sv.attach(42, 560, 2480);
    FLL_sv.attach(43, 560, 2480);
    FLF_sv.attach(44, 560, 2480);

    FRS_sv.attach(39, 560, 2480);
    FRL_sv.attach(40, 560, 2480);
    FRF_sv.attach(41, 560, 2480);

    RLS_sv.attach(48, 560, 2480);
    RLL_sv.attach(49, 560, 2480);
    RLF_sv.attach(50, 560, 2480);

    RRS_sv.attach(45, 560, 2480);
    RRL_sv.attach(46, 560, 2480);
    RRF_sv.attach(47, 560, 2480);
}

void loop(){
    FLS_sv.writeMicroseconds(FLS_angle);  FRS_sv.writeMicroseconds(FRS_angle);
    FLL_sv.writeMicroseconds(FLL_angle);  FRL_sv.writeMicroseconds(FRL_angle);
    FLF_sv.writeMicroseconds(FLF_angle);  FRF_sv.writeMicroseconds(FRF_angle);

    RLS_sv.writeMicroseconds(RLS_angle);  RRS_sv.writeMicroseconds(RRS_angle);
    RLL_sv.writeMicroseconds(RLL_angle);  RRL_sv.writeMicroseconds(RRL_angle);
    RLF_sv.writeMicroseconds(RLF_angle);  RRF_sv.writeMicroseconds(RRF_angle);

    nh.spinOnce();
    delay(1);
}
