#include <robot_control/pointMessages.h>

pointMessages::pointMessages(){
    
    jointStatesSub = n.subscribe("/arm_controller/state", 1, &pointMessages::jointStatesCallback, this);
    gazeSub = n.subscribe("/gazeHyps_rqt", 1, &pointMessages::gazeCallback, this);
    trajectoryStatusSub = n.subscribe("arm_controller/follow_joint_trajectory/status", 1, &pointMessages::trajectoryStatusCallback, this);
    
    jointStatesPub = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/arm_controller/follow_joint_trajectory/goal", 10);
    
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

void pointMessages::trajectoryStatusCallback(const actionlib_msgs::GoalStatusArray& trajStatus){
    
    // Compact message definition actionlib_msgs/GoalStatusArray
        // std_msgs/Header header
        // actionlib_msgs/GoalStatus[] status_list
    // Compact Message Definition actionlib_msgs/GoalStatus
        // uint8 PENDING=0
        // uint8 ACTIVE=1
        // uint8 PREEMPTED=2
        // uint8 SUCCEEDED=3
        // uint8 ABORTED=4
        // uint8 REJECTED=5
        // uint8 PREEMPTING=6
        // uint8 RECALLING=7
        // uint8 RECALLED=8
        // uint8 LOST=9
        // actionlib_msgs/GoalID goal_id
        // uint8 status
        // string text
    
    status = (double) trajStatus.status_list[0].status;
    std::cout << "<---------- Status ---------->" << std::endl;
    std::cout << status << std::endl;
    
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
        point.velocities.push_back(1);
    }
    point.time_from_start.nsec = 15e7;
    point.time_from_start.sec = 0;
                    
    goal_traj.goal.trajectory.points.push_back(point);
    
    //std::cout << "<----------point---------->" << std::endl;
    //std::cout << point << std::endl;
    
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
        std::cout << "Initializing position!" << std::endl;
//         while(status!=3){
//             ros::spinOnce;
//             ros::Duration(0.5).sleep();
//         }
//         std::cout << "In starting position!" << std::endl;
        //ros::Duration(10).sleep();
//         std::cout << "Ready!" << std::endl;
    }
}

void pointMessages::move(){
    bool skip = false;
    
    // gaze angles are in degrees, so a transformation to radians is needed
    horGaze = horGaze * M_PI/180;
    verGaze = verGaze * M_PI/180;
    
    // transformation from face to point on a sphere
    Eigen::MatrixXd T_fs;
    Eigen::MatrixXd R_yf = Eigen::MatrixXd::Zero(4,4);
    Eigen::MatrixXd R_xf = Eigen::MatrixXd::Zero(4,4);
    Eigen::MatrixXd T_d = Eigen::MatrixXd::Identity(4,4);
    
    R_yf(0,0) = cos(horGaze);
    R_yf(2,0) = -sin(horGaze);
    R_yf(1,1) = 1;
    R_yf(0,2) = sin(horGaze);
    R_yf(2,2) = cos(horGaze);
    R_yf(3,3) = 1;
    
    R_xf(0,0) = 1;
    R_xf(1,1) = cos(verGaze);
    R_xf(2,1) = sin(verGaze);
    R_xf(1,2) = -sin(verGaze);
    R_xf(2,2) = cos(verGaze);
    R_xf(3,3) = 1;
    
    T_d(2,3) = d;
    
    T_fs = R_yf * R_xf * T_d;
    
    // transformation from base to the center of the sphere
    Eigen::MatrixXd T_bf = Eigen::MatrixXd::Zero(4,4);
    
    // direct kinematics transformation matrix
    Eigen::MatrixXd T_06;
    T_06 = kinematic.directKinematics(lwa4p_temp_q, 6);
    
    // transformation to match the orientations
    Eigen::MatrixXd T_orient = Eigen::MatrixXd::Zero(4,4);
    
    T_orient(1,0) = 1;
    T_orient(0,1) = 1;
    T_orient(2,2) = -1;
    T_orient(3,3) = 1;
    
    T_bf = T_06 * T_d * T_orient;
    
    // transformation matrix from the coordinate system of the base to the coordinate system on the sphere
    Eigen::MatrixXd T;
    T = T_bf * T_fs;
    
    // final coordinate system orientation, z pointing in the direction opposite of the sphere normal
    T = T * T_orient;
    
    // create input vector for inverse kinematics
    Eigen::MatrixXd goal_w = Eigen::MatrixXd::Zero(9,1);
    
    goal_w(0,0) = T(0,3);
    goal_w(1,0) = T(1,3);
    goal_w(2,0) = T(2,3);
    goal_w(3,0) = T(0,0);
    goal_w(4,0) = T(1,0);
    goal_w(5,0) = T(2,0);
    goal_w(6,0) = T(0,2);
    goal_w(7,0) = T(1,2);
    goal_w(8,0) = T(2,2);
    
    Eigen::MatrixXd goal_q;
    
    //std::cout << "<----------Tool configuration vector (in mm)---------->" << std::endl;
    //std::cout << goal_w << std::endl;
    
    goal_q = kinematic.inverseKinematics(goal_w); // returns all possible solutions
//         std::cout << goal_q << std::endl;
    
    goal_q = kinematic.inverseKinematics_closestQ(goal_w, lwa4p_temp_q); // returns closest solution
//     std::cout << goal_q << std::endl;
    
    // check if nan appears in the solution, if it appears, ignore this result and repeat the procedure
    for (int i = 0; i < 6; i = i + 1){
        if (std::isnan(goal_q(i,0))) {
            skip = true;
            std::cout << "Inverse kinematics solution not feasible!" << std::endl;
            std::cout << "Check wanted position..." << std::endl;
            break;
        }
    }
    
    for (int i = 0; i < 6; i = i + 1){
        //std::cout << goal_q(i,0) - lwa4p_temp_q(i,0) << std::endl;
        if (std::abs(goal_q(i,0) - lwa4p_temp_q(i,0)) < 0.001){
            skip = true;
        }
    }
    
    if (!skip) {
        jointStatesPub.publish(goal_traj);
        T_start = ros::Time::now();
        goal_traj = {};
        prepareMsg(goal_q, T_start, goal_traj);
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
        
        if (status == 3) {
            move();
        }
        
        r.sleep();
    }
}