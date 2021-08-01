#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "fr_ik_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Set up the quarobo planning interface
    static const std::string FR_PLANNING_GROUP = "FR_ik";
    moveit::planning_interface::MoveGroupInterface fr_group(FR_PLANNING_GROUP);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // Prepare
    ROS_INFO("Moving to prepare pose");
    ROS_INFO_NAMED("quarobo", "Reference frame: %s", fr_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("quarobo", "End effector link: %s", fr_group.getEndEffectorLink().c_str());
    fr_group.setPlanningTime(0.05);
    fr_group.setPlannerId("RRTConnectkConfigDefault");
    fr_group.setGoalTolerance(0.01);

    // pose1
    geometry_msgs::PoseStamped pose1;
    pose1.header.frame_id = "base_link";
    //pose1.pose.position.x = 0.00;
    //pose1.pose.position.y = 0.00;
    pose1.pose.position.z = 0.00;
    pose1.pose.orientation.w = 1.0;

    // pose2
    geometry_msgs::PoseStamped pose2;
    pose2.header.frame_id = "base_link";
    //pose2.pose.position.x = 0.00;
    //pose2.pose.position.y = 0.00;
    pose2.pose.position.z += 0.01;
    pose2.pose.orientation.w = 1.0;


    // 機能しない
    //std::vector<double> jointValues = fr_group.getCurrentJointValues();
    //for(auto& i: jointValues) ROS_INFO("initial jointValues : %lf", i);

    moveit::planning_interface::MoveItErrorCode ret;

    ROS_INFO("move to walk_pose");
    fr_group.setNamedTarget("walk_pose");
    fr_group.plan(my_plan);
    ret = fr_group.move();
    if (!ret) {
        ROS_WARN("Fail: %i", ret.val);
    }
    ros::Duration(0.5).sleep();

    ROS_INFO("move to pose2");
    fr_group.setPoseTarget(pose2);
    fr_group.plan(my_plan);
    ret = fr_group.move();
    if (!ret) {
        ROS_WARN("Fail: %i", ret.val);
    }
    ros::Duration(0.5).sleep();

    ROS_INFO("move to pose1");
    fr_group.setPoseTarget(pose1);
    fr_group.plan(my_plan);
    ret = fr_group.move();
    if (!ret) {
        ROS_WARN("Fail: %i", ret.val);
    }
    ros::Duration(0.5).sleep();


    ROS_INFO("Finish!");
    ros::shutdown();
    return 0;
}
