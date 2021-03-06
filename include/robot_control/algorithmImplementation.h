#include <iostream>
#include <string>
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <queue>

// ROS
#include "ros/ros.h"
#include <std_msgs/Header.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <ros/callback_queue.h>

// user defined
#include "gazetool/GazeHyps.h"
#include "gazetool/GazeInfo.h"
#include <schunk_lwa4p_kinematics/lwa4p_kinematics.h>

class algorithmImplementation {
public:
    algorithmImplementation();
    ~algorithmImplementation();
    void run();
    
private: 
    // callbacks and member functions
    void jointStatesCallback(const control_msgs::JointTrajectoryControllerState& jointStates);
    void gazeCallback(const gazetool::GazeHyps& msg);
    void additionalGazetoolInformationCallback(const gazetool::GazeInfo& msg);
    void trajectoryStatusCallback(const actionlib_msgs::GoalStatusArray& trajStatus);
    void trajectoryResultCallback(const control_msgs::FollowJointTrajectoryActionResult& trajResult);
    
    void initializeBuffer();
    void initializeKinematics();
    void initializePosition();
    
    Eigen::MatrixXd calcJacobian(Eigen::MatrixXd lwa4p_temp_q);
    double wrapToPi(double angle);
    bool prepareMsg(Eigen::MatrixXd goal_q, ros::Time Tprep, control_msgs::FollowJointTrajectoryActionGoal& goal_traj);
    void trackGaze();
    
    // Node handle, publishers and subscribers
    ros::NodeHandle nh;
    ros::CallbackQueue result_queue;
    ros::Subscriber trajectoryResultSub;
    
    ros::NodeHandle n;
    ros::Subscriber jointStatesSub; // joint states
    ros::Subscriber gazeSub;
    ros::Subscriber trajectoryStatusSub;
    ros::Publisher jointStatesPub; // publish new joint states
    ros::Subscriber gazeInfoSub;
    
    // Algorithm parameters
    double d = 800; // distance from the camera to the observer
    double x0 = -500; // starting position
    double y0 = 0;
    double z0 = 800;
    
    //buffer
    std::queue<int> buffer;
    int bufferSum;
    int bufferSize = 10; // change if strickter conditions want to be met
    double threshold = 0.6 * bufferSize;
    
    // gaze data
    double horGaze, verGaze;
    bool firstMutGazeDetected = false;
    bool mutGaze;
    double horGazeTolerance; // information from gazetool GUI
    double verGazeTolerance;
    
    // joint states
    Eigen::MatrixXd lwa4p_temp_q;
    int DOF = 6;
    
    // lwa4p_kinematics
    lwa4p_kinematics kinematic;
    int robot_id = 1; // to choose blue robot (important when loading kinematic parameters)
    // bool inverse = false;
    bool jacobian = true;

    control_msgs::FollowJointTrajectoryActionGoal goal_traj;
    ros::Time T_start;
    double status = 0;
    double result = -1; // initial value, different than all the possible values result and error can take
    double error = 1;
    
};