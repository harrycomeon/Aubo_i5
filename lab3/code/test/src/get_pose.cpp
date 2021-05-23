#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int  main(int argc, char **argv)
{
    ros::init(argc,argv,"get_pose");
    ros::AsyncSpinner spin(1);
    spin.start();

    geometry_msgs::Pose get_current_pose;
    moveit::planning_interface::MoveGroupInterface aubo_arm("manipulator_i5");
    std::string reference = "base_link";

    aubo_arm.setPoseReferenceFrame(reference);

    get_current_pose = aubo_arm.getCurrentPose().pose;

    ROS_INFO_STREAM("current pose is "<<get_current_pose);

    ros::waitForShutdown();
    return 0;
}

