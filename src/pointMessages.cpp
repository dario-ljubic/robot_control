#include <robot_control/pointMessages.h>

pointMessages::pointMessages(){
    
    jointStatesSub = n.subscribe("/arm_controller/state", 1, &pointMessages::jointStatesCallback, this);
    gazeSub = n.subscribe("/gazeHyps_rqt", 1, &pointMessages::gazeCallback, this);
    
    jointStatesPub = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/arm_controller/follow_joint_trajectory/goal", 1);
    
}

pointMessages::~pointMessages(){
    
}

void pointMessages::jointStatesCallback(const control_msgs::JointTrajectoryControllerState& jointStates){
    
    lwa4p_temp_q = Eigen::MatrixXd::Zero(6, 1);
    
    std::vector<double> positions = jointStates.actual.positions;   
        for (unsigned int i = 0; i < positions.size(); i++)
        {
            lwa4p_temp_q(i,0) = positions[i];
        }
    
    //std::cout << lwa4p_temp_q << std::endl;
}

void pointMessages::gazeCallback(const gazetool::GazeHyps& msg){
    
    horGaze = msg.horGaze;
    verGaze = -msg.verGaze;
    std::cout << "<----------Gaze callback---------->" << std::endl;
    std::cout << "Horizontal gaze: " << horGaze << std::endl;
    std::cout << "Vertical gaze: " << verGaze << std::endl;
    
}

bool pointMessages::prepareMsg(Eigen::MatrixXd goal_q, ros::Time Tprep, control_msgs::FollowJointTrajectoryActionGoal& goal_traj){

    // Compact message definition -> FollowJointTrajectoryActionGoal  
        // std_msgs/Header header
        // actionlib_msgs/GoalID goal_id
        // control_msgs/FollowJointTrajectoryGoal goal
    // Compact message definition -> goal_id
        // time stamp
        // string id
    // Compact message definition -> FollowJointTrajectoryGoal
        // trajectory_msgs/JointTrajectory trajectory
        // control_msgs/JointTolerance[] path_tolerance
        // control_msgs/JointTolerance[] goal_tolerance
        // duration goal_time_tolerance
    // Compact message definition -> trajectory
        // std_msgs/Header header
        // string[] joint_names
        // trajectory_msgs/JointTrajectoryPoint[] points
    // Compact message definition -> header
        // uint32 seq
        // time stamp
        // string frame_id
    // Compact message definition -> points
        // float64[] positions
        // float64[] velocities
        // float64[] accelerations
        // duration time_from_start    
    
    // http://wiki.ros.org/joint_trajectory_controller
    // http://answers.ros.org/question/50610/the-meaning-of-velocities-accelerations-and-time_from_start-in-jointtrajectorypointmsg/
    
    std_msgs::Header h;
    h.stamp = ros::Time::now();
    std::ostringstream convert;
    convert << h.stamp.nsec;
    
    goal_traj.goal_id.stamp = h.stamp;
    goal_traj.goal_id.id = "robot_ctrl" + convert.str();
    
    // append joint names
    goal_traj.goal.trajectory.joint_names.push_back("arm_1_joint");
    goal_traj.goal.trajectory.joint_names.push_back("arm_2_joint");
    goal_traj.goal.trajectory.joint_names.push_back("arm_3_joint");
    goal_traj.goal.trajectory.joint_names.push_back("arm_4_joint");
    goal_traj.goal.trajectory.joint_names.push_back("arm_5_joint");
    goal_traj.goal.trajectory.joint_names.push_back("arm_6_joint");

    std_msgs::Header h2;
    h2.stamp.sec = 0; 
    h2.stamp.nsec = 0;
    
    goal_traj.goal.trajectory.header.stamp = h2.stamp;
    goal_traj.header.stamp = h2.stamp;
    
    // next, prepare a point to be sent
    trajectory_msgs::JointTrajectoryPoint point;
    
    for (int i = 0; i < DOF; i++)
    {
            
        point.positions.push_back(goal_q(i,0));
        point.velocities.push_back(0);
    }
    point.time_from_start.nsec = 15e7;
    point.time_from_start.sec = 0;
                    
    goal_traj.goal.trajectory.points.push_back(point);
    
    return 1;   
}

void pointMessages::initializeKinematics(){
    
    std::string configFile;
    std::string path = ros::package::getPath("schunk_lwa4p_kinematics");
    
    ros::NodeHandle private_node_handle_("~");

    //getting ros params
    private_node_handle_.param("config_file", configFile, path + std::string("/config/lwa4p_configuration.yaml"));
    
    kinematic.loadParameters(robot_id, configFile);
}

void pointMessages::initializePosition(){
    
    // ROS needs some time to register the core and to establish all subscriber connections.
    // Since only one message is sent in the beginning, it is lost. Therefore, loop until connection
    // is established. 
    ros::Rate poll_rate(100);
    while(jointStatesPub.getNumSubscribers() == 0)
        poll_rate.sleep();
    
    Eigen::MatrixXd goal_w = Eigen::MatrixXd::Zero(9, 1);
    Eigen::MatrixXd goal_q, q0;
    bool skip = false;
    
    goal_w(0,0) = x0;
    goal_w(1,0) = y0;
    goal_w(2,0) = z0;
    goal_w(3,0) = 0;
    goal_w(4,0) = 0;
    goal_w(5,0) = 1;
    goal_w(6,0) = 1;
    goal_w(7,0) = 0;
    goal_w(8,0) = 0;
    
    goal_q = kinematic.inverseKinematics(goal_w);
    //std::cout << goal_q << std::endl;
    
    q0 = Eigen::MatrixXd::Zero(6, 1);
    goal_q = kinematic.inverseKinematics_closestQ(goal_w, q0); // returns closest solution
    
    // check if nan appears in the solution, if it appears, ignore this result and repeat the procedure
    for (int i = 0; i < 6; i = i + 1){
        if (std::isnan(goal_q(i,0))) {
            skip = true;
            break;
        }
    }
    
    T_start = ros::Time::now();
    goal_traj = {};
    prepareMsg(goal_q, T_start, goal_traj);
    
    if (!skip) {
        jointStatesPub.publish(goal_traj);
        std::cout << "In starting position!" << std::endl;
        ros::Duration(5).sleep();
        std::cout << "Ready!" << std::endl;
    }
}

void pointMessages::run(){
    
    T_start.sec = 0;
    T_start.nsec = 0;
    
    initializeKinematics();
    
    initializePosition();
    
    ros::Rate r(1); // rate is set to one because algorithm is tested much easier
    while(ros::ok()){
        
        ros::spinOnce();
       
        std::cout << "Entered run" << std::endl;
        
        r.sleep();
    }
}