#include <iostream>
#include <string>
#include <math.h>
#include <eigen3/Eigen/Eigen>

// ROS
#include "ros/ros.h"
#include <std_msgs/Header.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>

// user defined
#include "gazetool/GazeHyps.h"
#include <schunk_lwa4p_kinematics/lwa4p_kinematics.h>

class pointMessages {
public:
    pointMessages();
    ~pointMessages();
    void run();
    
private: 
    // callbacks and member functions
    void jointStatesCallback(const control_msgs::JointTrajectoryControllerState& jointStates);
    void gazeCallback(const gazetool::GazeHyps& msg);
    void trajectoryStatusCallback(const actionlib_msgs::GoalStatusArray& trajStatus);
    void initializeKinematics();
    void initializePosition();
    bool prepareMsg(Eigen::MatrixXd goal_q, ros::Time Tprep, control_msgs::FollowJointTrajectoryActionGoal& goal_traj);
    void move();
    
    // Node handle, publishers and subscriber
    ros::NodeHandle n;
    ros::Subscriber jointStatesSub; // joint states
    ros::Subscriber gazeSub; // gaze direction from rqt
    ros::Subscriber trajectoryStatusSub;
    ros::Publisher jointStatesPub; // publish new joint states
    
    // Algorithm parameters
    double d = 300; // distance from the camera to the observer
    double x0 = 500; // starting position
    double y0 = 0;
    double z0 = 1000;
    
    // gaze data
    double horGaze, verGaze;
    
    // joint states
    Eigen::MatrixXd lwa4p_temp_q;
    int DOF = 6;
    
    // lwa4p_kinematics
    lwa4p_kinematics kinematic;
    int robot_id = 0; // to choose blue robot (important when loading kinematic parameters)
    
    // path planning. In this case point to point movement
    control_msgs::FollowJointTrajectoryActionGoal goal_traj;
    ros::Time T_start;
    double status = 0;
};