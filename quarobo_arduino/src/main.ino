#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <Servo.h>

ros::NodeHandle nh;

sensor_msgs::JointState js;
ros::Publisher chatter("chatter", &js);

//std_msgs::String str_msg;
//ros::Publisher chatter("chatter", &str_msg);

Servo myservo;

void CallBack(const sensor_msgs::JointState& msg){
    js = msg;
    chatter.publish(&js);
    myservo.write(0);
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    myservo.write(90);
}

ros::Subscriber<sensor_msgs::JointState> sub("joint_states", &CallBack);

char hello[] = "Hello!";

void setup(){
    pinMode(13, OUTPUT);
    nh.initNode();
    nh.advertise(chatter);
    nh.subscribe(sub);

    myservo.attach(6);
    myservo.write(90);
}

void loop(){
//    str_msg.data = hello;
//    chatter.publish(&str_msg);
    nh.spinOnce();
    delay(1);
}
