#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "moveit_commander");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Set up the quarobo planning interface
    static const std::string FR_PLANNING_GROUP = "FR_ik";
    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface fr_group(FR_PLANNING_GROUP);
    //const robot_state::JointModelGroup* joint_model_group =
    //    fr_group.getCurrentState()->getJointModelGroup(FR_PLANNING_GROUP);

    //namespace rvt = rviz_visual_tools;
    //moveit_visual_tools::MoveItVisualTools visual_tools("FRT_link");
    //visual_tools.deleteAllMarkers();

    //visual_tools.loadRemoteControl();

    //Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    //text_pose.translation().z() = 0.029427;
    //visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    // Prepare
    ROS_INFO("Moving to prepare pose");
    ROS_INFO_NAMED("tutorial", "Reference frame: %s", fr_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", fr_group.getEndEffectorLink().c_str());
    fr_group.setPlanningTime(0.050);
    fr_group.setPlannerId("RRTConnect");
    fr_group.setGoalTolerance(0.01);

    // pose1
    geometry_msgs::PoseStamped pose1;
    pose1.header.frame_id = "FRT_link";
    pose1.pose.position.x = -0.059191;
    pose1.pose.position.y = -0.062673;
    pose1.pose.position.z = 0.029157;
    pose1.pose.orientation.x = 0.59244;
    pose1.pose.orientation.y = 0.040214;
    pose1.pose.orientation.z = -0.02963;
    pose1.pose.orientation.w = 0.80406;

    moveit::planning_interface::MoveItErrorCode ret;


    ROS_INFO("move to WP1");
    fr_group.setPoseReferenceFrame("FRT_link");
    fr_group.setNamedTarget("walk_pose");
    //fr_group.setPoseTarget(pose1);
    //ret = fr_group.move();
    //if (!ret) {
    //    ROS_WARN("Fail: %i", ret.val);
    //}
    fr_group.move();

    ros::shutdown();
    return 0;
}
