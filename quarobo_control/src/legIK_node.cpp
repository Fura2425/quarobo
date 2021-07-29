#include <ros/ros.h>

int main(int argc char **argv){
    ros::init(argc, argv, "ik_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();


    return 0;
}
