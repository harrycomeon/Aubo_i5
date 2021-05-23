#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <cstdlib>

int main(int argc,char** argv)
{
    ros::init(argc, argv, "move_aubo");
    ros::NodeHandle n;
    ros::AsyncSpinner spin(1);
    spin.start();
    moveit::planning_interface::MoveGroupInterface move_group("manipulator_i5");
    std::string reference="world";
    move_group.setPoseReferenceFrame(reference);
    move_group.allowReplanning(true);
    move_group.setMaxVelocityScalingFactor(0.4);
    move_group.setMaxAccelerationScalingFactor(0.3);
    move_group.setGoalOrientationTolerance(0.1);
    move_group.setGoalPositionTolerance(0.1);
    geometry_msgs::Pose target_pose;
    geometry_msgs::Pose current_pose;

//    move_group.setNamedTarget("home");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setPlanningTime(3.0);
    // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // if(success)
    // {
    //     move_group.execute(my_plan);
    //     sleep(1.0);
    //     ROS_INFO_STREAM("initial success is get");
    // }

    current_pose = move_group.getCurrentPose().pose;

    ROS_INFO_STREAM("current pose is "<<current_pose);
    double x,y,z,go_back,go_home;
    n.getParam("position_x",x);
    n.getParam("position_y",y);
    n.getParam("position_z",z);
    n.getParam("go_back",go_back);
    n.getParam("go_home",go_home);

    target_pose.position.x = current_pose.position.x;
    target_pose.position.y = current_pose.position.y;
    target_pose.position.z = current_pose.position.z - 0.1;
    target_pose.orientation.x = current_pose.orientation.x;
    target_pose.orientation.y = current_pose.orientation.y;
    target_pose.orientation.z = current_pose.orientation.z;
    target_pose.orientation.w = current_pose.orientation.w;

    ROS_INFO_STREAM("target_pose is "<<target_pose);
    std::vector<geometry_msgs::Pose> way_points;
    move_group.setPoseTarget(target_pose);
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success)
    {
        move_group.execute(my_plan);
        sleep(1);
        ROS_INFO_STREAM("success");
    }
    else
    {
        ROS_INFO_STREAM("this is wrong");
    }

    // current_pose.position.x = -0.48;
    // current_pose.position.y = -0.13;
    // current_pose.position.z = 0.45;
    // way_points.push_back(current_pose);
    // current_pose.position.x = 0.0;
    // current_pose.position.y = -0.21;
    // current_pose.position.z = 1.5;
    // way_points.push_back(current_pose);
    // current_pose.position.x = -0.48;
    // current_pose.position.y = -0.14;
    // current_pose.position.z = 0.95;
    // way_points.push_back(current_pose);
    // current_pose.position.x = 0.0;
    // current_pose.position.y = 0.21;
    // current_pose.position.z = 0.65;
    // way_points.push_back(current_pose);
    


    // moveit_msgs::RobotTrajectory trajectory;
    // const double threshold = 0.0;
    // const double eef = 0.01;
    // double fraction = 0.0;
    // int max_try = 100;
    // int attempts = 0;

    // while(fraction<1 && attempts<max_try)
    // {
    //     fraction = move_group.computeCartesianPath(way_points,eef,threshold,trajectory);
    //     attempts++;
    // }
    // if(fraction==1)
    // {
    //     ROS_INFO_STREAM("way_points is success");
    //     moveit::planning_interface::MoveGroupInterface::Plan this_plan;
    //     this_plan.trajectory_ = trajectory;
    //     move_group.execute(this_plan);
    //     sleep(1);
    //     current_pose = move_group.getCurrentPose().pose;
    //     ROS_INFO_STREAM("way_points currents pose is "<<current_pose);
    // }
    // else
    // {
    //     ROS_INFO_STREAM("waypoints is failed "<<fraction);
    // }

    // move_group.setPoseTarget(target_pose);
    // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // if(success)
    // {
    //     ROS_INFO_STREAM("go_target is success");
    //     current_pose = move_group.getCurrentPose().pose;
    //     ROS_INFO_STREAM("back current pose is "<<current_pose);
    // }
    // else
    // {
    //     ROS_INFO_STREAM("wrong");
    // }



    // move_group.setPlanningTime(5.0);
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // move_group.setPoseTarget(target_pose);
    // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // if (success)
    // {
    //     move_group.execute(my_plan);
    //     sleep(1);
    //     ROS_INFO_STREAM("success is get");
    // }
    // else
    // {
    //     ROS_INFO_STREAM("success is "<<success);
    // }

    // if(go_back)
    // {
    //     move_group.setPoseTarget(current_pose);
    //     bool success = move_group.plan(my_plan) == (moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //     if(success)
    //     {
    //         move_group.execute(my_plan);
    //         sleep(1);
    //         ROS_INFO_STREAM("goback success is get");
    //     }
    //     else
    //     {
    //         ROS_INFO_STREAM("goback success is "<<success);
    //     }
    // }

    // if(go_home)
    // {
    //     move_group.setNamedTarget("home");
    //     bool success = move_group.plan(my_plan) == (moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //     if(success)
    //     {
    //         move_group.execute(my_plan);
    //         sleep(1);
    //         ROS_INFO_STREAM("gohome success is get");
    //         current_pose = move_group.getCurrentPose().pose;
    //         ROS_INFO_STREAM("home pose is "<<current_pose);
    //     }
    //     else
    //     {
    //         ROS_INFO_STREAM("gohome success is "<<success);
    //     }
    // }

    ros::waitForShutdown();
    return 0;
}

