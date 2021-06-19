#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "moveit_commander");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Set up the quarobo planning interface
    static const std::string FR_PLANNING_GROUP = "FR_ik";
    moveit::planning_interface::MoveGroupInterface fr_group(FR_PLANNING_GROUP);

    // Prepare
    ROS_INFO("Moving to prepare pose");
    ROS_INFO_NAMED("quarobo", "Reference frame: %s", fr_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("quarobo", "End effector link: %s", fr_group.getEndEffectorLink().c_str());
    fr_group.setPlanningTime(0.050);
    fr_group.setPlannerId("RRTConnect");
    fr_group.setGoalTolerance(0.01);

    ROS_INFO("get current joint values");
    std::vector<double> jointValues = fr_group.getCurrentJointValues();
    for(auto& joint: jointValues) ROS_INFO("initial jointValues : %lf", joint);
    std::vector<double> home_posiValues = {0.0, 0.0, 0.0};

    // pose1
    //geometry_msgs::PoseStamped pose1;
    //pose1.header.frame_id = "FRT_link";
    //pose1.pose.position.x = -0.059191;
    //pose1.pose.position.y = -0.062673;
    //pose1.pose.position.z = 0.029157;
    //pose1.pose.orientation.x = 0.59244;
    //pose1.pose.orientation.y = 0.040214;
    //pose1.pose.orientation.z = -0.02963;
    //pose1.pose.orientation.w = 0.80406;

    moveit::planning_interface::MoveItErrorCode ret;

    //fr_group.setPoseReferenceFrame("FRT_link");
    //fr_group.setPoseTarget(pose1);

    ROS_INFO("move to WP1");
    fr_group.setNamedTarget("walk_pose");
    ret = fr_group.move();
    if (!ret) {
        ROS_WARN("Fail: %i", ret.val);
    }
    jointValues = fr_group.getCurrentJointValues();
    for(auto& joint: jointValues) ROS_INFO("WP1 jointValues : %lf", joint);
    ros::Duration(0.5).sleep();

    ROS_INFO("move to WP2");
    fr_group.setJointValueTarget(home_posiValues);
    ret = fr_group.move();
    if (!ret) {
        ROS_WARN("Fail: %i", ret.val);
    }
    jointValues = fr_group.getCurrentJointValues();
    for(auto& joint: jointValues) ROS_INFO("WP2 jointValues : %lf", joint);
    ros::Duration(0.5).sleep();

    ROS_INFO("Finish!");
    ros::shutdown();
    return 0;
}
