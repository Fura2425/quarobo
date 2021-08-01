#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "moveit_commander");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Set up the fr_group planning interface
    moveit::planning_interface::MoveGroupInterface fr_group("FR_ik");

    // Prepare
    ROS_INFO("Moving to prepare pose");
    ROS_INFO_NAMED("tutorial", "Reference frame: %s", fr_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", fr_group.getEndEffectorLink().c_str());
    fr_group.setPlanningTime(0.050);
    fr_group.setPlannerId("RRTConnect");
    fr_group.setGoalTolerance(0.01);

    // pose1
    geometry_msgs::PoseStamped pose1;
    pose1.header.frame_id = "base_link";
    pose1.pose.position.x = 0.60;
    pose1.pose.position.y = 0.40;
    pose1.pose.position.z = 0.30;
    //pose1.pose.orientation.w = 1.0;

    // pose2
    geometry_msgs::PoseStamped pose2;
    pose2.header.frame_id = "base_link";
    pose2.pose.position.x = 0.70;
    pose2.pose.position.y = 0.50;
    pose2.pose.position.z = 0.40;
    //pose2.pose.orientation.w = 1.0;

    // pose3
    geometry_msgs::PoseStamped pose3;
    pose3.header.frame_id = "base_link";
    pose3.pose.position.x = 0.60;
    pose3.pose.position.y = 0.40;
    pose3.pose.position.z = 0.30;
    //pose3.pose.orientation.w = 1.0;

    // pose4
    geometry_msgs::PoseStamped pose4;
    pose4.header.frame_id = "base_link";
    pose4.pose.position.x = 0.70;
    pose4.pose.position.y = 0.50;
    pose4.pose.position.z = 0.40;
    //pose4.pose.orientation.w = 1.0;

    moveit::planning_interface::MoveItErrorCode ret;

    ROS_INFO("move to WP1");
    fr_group.setPoseTarget(pose1);
    ret = fr_group.move();
    if (!ret) {
        ROS_WARN("Fail: %i", ret.val);
    }
    ros::Duration(0.5).sleep();

    ROS_INFO("move to WP2");
    fr_group.setPoseTarget(pose2);
    ret = fr_group.move();
    if (!ret) {
        ROS_WARN("Fail: %i", ret.val);
    }
    ros::Duration(0.5).sleep();

    ROS_INFO("move to WP3");
    fr_group.setPoseTarget(pose3);
    ret = fr_group.move();
    if (!ret) {
        ROS_WARN("Fail: %i", ret.val);
    }
    ros::Duration(0.5).sleep();

    ROS_INFO("move to WP4");
    fr_group.setPoseTarget(pose4);
    ret = fr_group.move();
    if (!ret) {
        ROS_WARN("Fail: %i", ret.val);
    }
    ros::Duration(0.5).sleep();

    ros::shutdown();
    return 0;
}
