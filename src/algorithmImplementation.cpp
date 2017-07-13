#include <robot_control/algorithmImplementation.h>

algorithmImplementation::algorithmImplementation(){
    
    // ROS default callback queue
    jointStatesSub = n.subscribe("/arm_controller/state", 1, &algorithmImplementation::jointStatesCallback, this);
    gazeInfoSub = n.subscribe("additionalGazetoolInformation", 1, &algorithmImplementation::additionalGazetoolInformationCallback, this);
    gazeSub = n.subscribe("gazeHyps_filtered", 1, &algorithmImplementation::gazeCallback, this);
    trajectoryStatusSub = n.subscribe("arm_controller/follow_joint_trajectory/status", 1, &algorithmImplementation::trajectoryStatusCallback, this);
    jointStatesPub = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/arm_controller/follow_joint_trajectory/goal", 1);
    
    // user defined callback queue
    nh.setCallbackQueue(&result_queue);
    trajectoryResultSub = nh.subscribe("arm_controller/follow_joint_trajectory/result", 1, &algorithmImplementation::trajectoryResultCallback, this);
    
}

algorithmImplementation::~algorithmImplementation(){
    
}

void algorithmImplementation::jointStatesCallback(const control_msgs::JointTrajectoryControllerState& jointStates){
    
    lwa4p_temp_q = Eigen::MatrixXd::Zero(6, 1);
    
    std::vector<double> positions = jointStates.actual.positions;   
        for (unsigned int i = 0; i < positions.size(); i++)
        {
            lwa4p_temp_q(i,0) = positions[i];
        }
    
    //std::cout << lwa4p_temp_q << std::endl;
}

void algorithmImplementation::gazeCallback(const gazetool::GazeHyps& msg){
    
    horGaze = -msg.verGaze;
    verGaze = -msg.horGaze;
    mutGaze = msg.mutGaze;
    
    buffer.push(mutGaze);
    bufferSum = bufferSum - buffer.front() + mutGaze;
    buffer.pop();
    //std::cout << "<----------Gaze callback---------->" << std::endl;
    //std::cout << "Horizontal gaze: " << horGaze << std::endl;
    //std::cout << "Vertical gaze: " << verGaze << std::endl;
}

void algorithmImplementation::additionalGazetoolInformationCallback(const gazetool::GazeInfo& msg){
    horGazeTolerance = msg.horizontalGazeTolerance;
    verGazeTolerance = msg.verticalGazeTolerance;
    
}

void algorithmImplementation::trajectoryResultCallback(const control_msgs::FollowJointTrajectoryActionResult& trajResult){
    
    // Compact message definition control_msgs/FollowJointTrajectoryActionResult
        // std_msgs/Header header
        // actionlib_msgs/GoalStatus status
        // control_msgs/FollowJointTrajectoryResult result
    
    // std::cout << "<---------- Result ---------->" << std::endl;
    //std::cout << (double) trajResult.status.status << std::endl;
    // std::cout << trajResult.result << std::endl;
    result = trajResult.status.status;
    error = trajResult.result.error_code;
}

void algorithmImplementation::trajectoryStatusCallback(const actionlib_msgs::GoalStatusArray& trajStatus){  
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
    // std::cout << "<---------- Status ---------->" << std::endl;
    // std::cout << status << std::endl;
}

Eigen::MatrixXd algorithmImplementation::calcJacobian(Eigen::MatrixXd lwa4p_temp_q){
    
    Eigen::MatrixXd jacob;
    jacob = Eigen::MatrixXd::Zero(9, 6);

    double l1, l2, l3, l4;
    if (robot_id == 0) { // for the blue robot
        l1 = 651;
        l2 = 369.2;
        l3 = 316;
        l4 = 85;
    }
    if (robot_id == 1){ // for the red robot
        l1 = 651;
        l2 = 500;
        l3 = 365;
        l4 = 85;
    }
    
    double q1, q2, q3, q4, q5, q6;
    q1 = lwa4p_temp_q(0,0);
    q2 = lwa4p_temp_q(1,0); 
    q3 = -lwa4p_temp_q(2,0); // the axis is flipped
    q4 = lwa4p_temp_q(3,0);
    q5 = lwa4p_temp_q(4,0);
    q6 = lwa4p_temp_q(5,0);
    
    jacob(0,0) = l2*sin(q1)*sin(q2) + l3*cos(q2)*sin(q1)*sin(q3) + l3*cos(q3)*sin(q1)*sin(q2) - l4*cos(q1)*sin(q4)*sin(q5) + l4*cos(q2)*cos(q5)*sin(q1)*sin(q3) + l4*cos(q3)*cos(q5)*sin(q1)*sin(q2) - l4*cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) + l4*cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5);
    jacob(0,1) = l3*cos(q1)*sin(q2)*sin(q3) - l3*cos(q1)*cos(q2)*cos(q3) - l2*cos(q1)*cos(q2) - l4*cos(q1)*cos(q2)*cos(q3)*cos(q5) + l4*cos(q1)*cos(q5)*sin(q2)*sin(q3) - l4*cos(q1)*cos(q2)*cos(q4)*sin(q3)*sin(q5) - l4*cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q5);
    jacob(0,2) = l3*cos(q1)*sin(q2)*sin(q3) - l3*cos(q1)*cos(q2)*cos(q3) - l4*cos(q1)*cos(q2)*cos(q3)*cos(q5) + l4*cos(q1)*cos(q5)*sin(q2)*sin(q3) - l4*cos(q1)*cos(q2)*cos(q4)*sin(q3)*sin(q5) - l4*cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q5);
    jacob(0,3) = l4*cos(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - l4*cos(q1)*cos(q2)*cos(q3)*sin(q4)*sin(q5) - l4*cos(q4)*sin(q1)*sin(q5);
    jacob(0,4) = l4*cos(q1)*cos(q2)*sin(q3)*sin(q5) - l4*cos(q5)*sin(q1)*sin(q4) + l4*cos(q1)*cos(q3)*sin(q2)*sin(q5) - l4*cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3) + l4*cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5);
    jacob(0,5) = 0;

    jacob(1,0) = l4*cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - l3*cos(q1)*cos(q2)*sin(q3) - l3*cos(q1)*cos(q3)*sin(q2) - l4*sin(q1)*sin(q4)*sin(q5) - l4*cos(q1)*cos(q2)*cos(q5)*sin(q3) - l4*cos(q1)*cos(q3)*cos(q5)*sin(q2) - l2*cos(q1)*sin(q2) - l4*cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5);
    jacob(1,1) = l3*sin(q1)*sin(q2)*sin(q3) - l3*cos(q2)*cos(q3)*sin(q1) - l2*cos(q2)*sin(q1) - l4*cos(q2)*cos(q3)*cos(q5)*sin(q1) + l4*cos(q5)*sin(q1)*sin(q2)*sin(q3) - l4*cos(q2)*cos(q4)*sin(q1)*sin(q3)*sin(q5) - l4*cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q5);
    jacob(1,2) = l3*sin(q1)*sin(q2)*sin(q3) - l3*cos(q2)*cos(q3)*sin(q1) - l4*cos(q2)*cos(q3)*cos(q5)*sin(q1) + l4*cos(q5)*sin(q1)*sin(q2)*sin(q3) - l4*cos(q2)*cos(q4)*sin(q1)*sin(q3)*sin(q5) - l4*cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q5);
    jacob(1,3) =  l4*cos(q1)*cos(q4)*sin(q5) - l4*cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q5) + l4*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5);
    jacob(1,4) = l4*cos(q1)*cos(q5)*sin(q4) + l4*cos(q2)*sin(q1)*sin(q3)*sin(q5) + l4*cos(q3)*sin(q1)*sin(q2)*sin(q5) + l4*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) - l4*cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3);
    jacob(1,5) = 0;

    jacob(2,0) = 0;    
    jacob(2,1) = l4*cos(q2)*cos(q3)*cos(q4)*sin(q5) - l3*cos(q2)*sin(q3) - l3*cos(q3)*sin(q2) - l4*cos(q2)*cos(q5)*sin(q3) - l4*cos(q3)*cos(q5)*sin(q2) - l2*sin(q2) - l4*cos(q4)*sin(q2)*sin(q3)*sin(q5);
    jacob(2,2) = l4*cos(q2)*cos(q3)*cos(q4)*sin(q5) - l3*cos(q3)*sin(q2) - l4*cos(q2)*cos(q5)*sin(q3) - l4*cos(q3)*cos(q5)*sin(q2) - l3*cos(q2)*sin(q3) - l4*cos(q4)*sin(q2)*sin(q3)*sin(q5);
    jacob(2,3) = -l4*cos(q2)*sin(q3)*sin(q4)*sin(q5) - l4*cos(q3)*sin(q2)*sin(q4)*sin(q5);
    jacob(2,4) = l4*sin(q2)*sin(q3)*sin(q5) - l4*cos(q2)*cos(q3)*sin(q5) + l4*cos(q2)*cos(q4)*cos(q5)*sin(q3) + l4*cos(q3)*cos(q4)*cos(q5)*sin(q2);
    jacob(2,5) = 0;
    
    jacob(3,0) = sin(q6)*(cos(q1)*cos(q4) - cos(q2)*cos(q3)*sin(q1)*sin(q4) + sin(q1)*sin(q2)*sin(q3)*sin(q4)) + cos(q6)*(cos(q1)*cos(q5)*sin(q4) + cos(q2)*sin(q1)*sin(q3)*sin(q5) + cos(q3)*sin(q1)*sin(q2)*sin(q5) + cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) - cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3));
    jacob(3,1) = cos(q6)*(cos(q1)*sin(q2)*sin(q3)*sin(q5) - cos(q1)*cos(q2)*cos(q3)*sin(q5) + cos(q1)*cos(q2)*cos(q4)*cos(q5)*sin(q3) + cos(q1)*cos(q3)*cos(q4)*cos(q5)*sin(q2)) - sin(q6)*(cos(q1)*cos(q2)*sin(q3)*sin(q4) + cos(q1)*cos(q3)*sin(q2)*sin(q4));
    jacob(3,2) = cos(q6)*(cos(q1)*sin(q2)*sin(q3)*sin(q5) - cos(q1)*cos(q2)*cos(q3)*sin(q5) + cos(q1)*cos(q2)*cos(q4)*cos(q5)*sin(q3) + cos(q1)*cos(q3)*cos(q4)*cos(q5)*sin(q2)) - sin(q6)*(cos(q1)*cos(q2)*sin(q3)*sin(q4) + cos(q1)*cos(q3)*sin(q2)*sin(q4));
    jacob(3,3) = cos(q6)*(cos(q4)*cos(q5)*sin(q1) + cos(q1)*cos(q2)*cos(q3)*cos(q5)*sin(q4) - cos(q1)*cos(q5)*sin(q2)*sin(q3)*sin(q4)) - sin(q6)*(sin(q1)*sin(q4) - cos(q1)*cos(q2)*cos(q3)*cos(q4) + cos(q1)*cos(q4)*sin(q2)*sin(q3));
    jacob(3,4) = -cos(q6)*(sin(q1)*sin(q4)*sin(q5) + cos(q1)*cos(q2)*cos(q5)*sin(q3) + cos(q1)*cos(q3)*cos(q5)*sin(q2) - cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5));
    jacob(3,5) = cos(q6)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*cos(q3)*sin(q4) - cos(q1)*sin(q2)*sin(q3)*sin(q4)) + sin(q6)*(cos(q1)*cos(q2)*sin(q3)*sin(q5) - cos(q5)*sin(q1)*sin(q4) + cos(q1)*cos(q3)*sin(q2)*sin(q5) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5) - cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3));
    
    jacob(4,0) = sin(q6)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*cos(q3)*sin(q4) - cos(q1)*sin(q2)*sin(q3)*sin(q4)) - cos(q6)*(cos(q1)*cos(q2)*sin(q3)*sin(q5) - cos(q5)*sin(q1)*sin(q4) + cos(q1)*cos(q3)*sin(q2)*sin(q5) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5) - cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3));
    jacob(4,1) = cos(q6)*(sin(q1)*sin(q2)*sin(q3)*sin(q5) - cos(q2)*cos(q3)*sin(q1)*sin(q5) + cos(q2)*cos(q4)*cos(q5)*sin(q1)*sin(q3) + cos(q3)*cos(q4)*cos(q5)*sin(q1)*sin(q2)) - sin(q6)*(cos(q2)*sin(q1)*sin(q3)*sin(q4) + cos(q3)*sin(q1)*sin(q2)*sin(q4));
    jacob(4,2) = cos(q6)*(sin(q1)*sin(q2)*sin(q3)*sin(q5) - cos(q2)*cos(q3)*sin(q1)*sin(q5) + cos(q2)*cos(q4)*cos(q5)*sin(q1)*sin(q3) + cos(q3)*cos(q4)*cos(q5)*sin(q1)*sin(q2)) - sin(q6)*(cos(q2)*sin(q1)*sin(q3)*sin(q4) + cos(q3)*sin(q1)*sin(q2)*sin(q4));
    jacob(4,3) = sin(q6)*(cos(q1)*sin(q4) + cos(q2)*cos(q3)*cos(q4)*sin(q1) - cos(q4)*sin(q1)*sin(q2)*sin(q3)) - cos(q6)*(cos(q1)*cos(q4)*cos(q5) - cos(q2)*cos(q3)*cos(q5)*sin(q1)*sin(q4) + cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q4));
    jacob(4,4) = -cos(q6)*(cos(q2)*cos(q5)*sin(q1)*sin(q3) - cos(q1)*sin(q4)*sin(q5) + cos(q3)*cos(q5)*sin(q1)*sin(q2) - cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) + cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5));
    jacob(4,5) = sin(q6)*(cos(q1)*cos(q5)*sin(q4) + cos(q2)*sin(q1)*sin(q3)*sin(q5) + cos(q3)*sin(q1)*sin(q2)*sin(q5) + cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) - cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)) - cos(q6)*(cos(q1)*cos(q4) - cos(q2)*cos(q3)*sin(q1)*sin(q4) + sin(q1)*sin(q2)*sin(q3)*sin(q4));

    jacob(5,0) = 0;
    jacob(5,1) = cos(q2 + q3)*sin(q4)*sin(q6) - cos(q6)*(cos(q2)*sin(q3)*sin(q5) + cos(q3)*sin(q2)*sin(q5) + cos(q2)*cos(q3)*cos(q4)*cos(q5) - cos(q4)*cos(q5)*sin(q2)*sin(q3));
    jacob(5,2) = cos(q2 + q3)*sin(q4)*sin(q6) - cos(q6)*(cos(q2)*sin(q3)*sin(q5) + cos(q3)*sin(q2)*sin(q5) + cos(q2)*cos(q3)*cos(q4)*cos(q5) - cos(q4)*cos(q5)*sin(q2)*sin(q3));
    jacob(5,3) = cos(q6)*(cos(q2)*cos(q5)*sin(q3)*sin(q4) + cos(q3)*cos(q5)*sin(q2)*sin(q4)) + sin(q2 + q3)*cos(q4)*sin(q6);
    jacob(5,4) = cos(q6)*(cos(q2)*cos(q3)*cos(q5) - cos(q5)*sin(q2)*sin(q3) + cos(q2)*cos(q4)*sin(q3)*sin(q5) + cos(q3)*cos(q4)*sin(q2)*sin(q5));
    jacob(5,5) = sin(q6)*(sin(q2)*sin(q3)*sin(q5) - cos(q2)*cos(q3)*sin(q5) + cos(q2)*cos(q4)*cos(q5)*sin(q3) + cos(q3)*cos(q4)*cos(q5)*sin(q2)) + sin(q2 + q3)*cos(q6)*sin(q4);
    
    jacob(6,0) = cos(q2)*cos(q5)*sin(q1)*sin(q3) - cos(q1)*sin(q4)*sin(q5) + cos(q3)*cos(q5)*sin(q1)*sin(q2) - cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) + cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5);
    jacob(6,1) = cos(q1)*cos(q5)*sin(q2)*sin(q3) - cos(q1)*cos(q2)*cos(q3)*cos(q5) - cos(q1)*cos(q2)*cos(q4)*sin(q3)*sin(q5) - cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q5);
    jacob(6,2) = cos(q1)*cos(q5)*sin(q2)*sin(q3) - cos(q1)*cos(q2)*cos(q3)*cos(q5) - cos(q1)*cos(q2)*cos(q4)*sin(q3)*sin(q5) - cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q5);
    jacob(6,3) = cos(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - cos(q1)*cos(q2)*cos(q3)*sin(q4)*sin(q5) - cos(q4)*sin(q1)*sin(q5);
    jacob(6,4) = cos(q1)*cos(q2)*sin(q3)*sin(q5) - cos(q5)*sin(q1)*sin(q4) + cos(q1)*cos(q3)*sin(q2)*sin(q5) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5) - cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3);
    jacob(6,5) = 0;

    jacob(7,0) = cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - cos(q1)*cos(q2)*cos(q5)*sin(q3) - cos(q1)*cos(q3)*cos(q5)*sin(q2) - sin(q1)*sin(q4)*sin(q5) - cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5);
    jacob(7,1) = cos(q5)*sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*cos(q5)*sin(q1) - cos(q2)*cos(q4)*sin(q1)*sin(q3)*sin(q5) - cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q5);
    jacob(7,2) = cos(q5)*sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*cos(q5)*sin(q1) - cos(q2)*cos(q4)*sin(q1)*sin(q3)*sin(q5) - cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q5);
    jacob(7,3) = cos(q1)*cos(q4)*sin(q5) - cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q5) + sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5);
    jacob(7,4) = cos(q1)*cos(q5)*sin(q4) + cos(q2)*sin(q1)*sin(q3)*sin(q5) + cos(q3)*sin(q1)*sin(q2)*sin(q5) + cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) - cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3);
    jacob(7,5) = 0;

    jacob(8,0) = 0;
    jacob(8,1) = cos(q2)*cos(q3)*cos(q4)*sin(q5) - cos(q3)*cos(q5)*sin(q2) - cos(q2)*cos(q5)*sin(q3) - cos(q4)*sin(q2)*sin(q3)*sin(q5);
    jacob(8,2) = cos(q2)*cos(q3)*cos(q4)*sin(q5) - cos(q3)*cos(q5)*sin(q2) - cos(q2)*cos(q5)*sin(q3) - cos(q4)*sin(q2)*sin(q3)*sin(q5);
    jacob(8,3) = -cos(q2)*sin(q3)*sin(q4)*sin(q5) - cos(q3)*sin(q2)*sin(q4)*sin(q5);
    jacob(8,4) = sin(q2)*sin(q3)*sin(q5) - cos(q2)*cos(q3)*sin(q5) + cos(q2)*cos(q4)*cos(q5)*sin(q3) + cos(q3)*cos(q4)*cos(q5)*sin(q2);
    jacob(8,5) = 0;
    
    return jacob;
}

template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

bool algorithmImplementation::prepareMsg(Eigen::MatrixXd goal_q, ros::Time Tprep, control_msgs::FollowJointTrajectoryActionGoal& goal_traj){

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
        point.velocities.push_back(1); // WARNING: remove magic number
    }
    point.time_from_start.nsec = 15e7;
    point.time_from_start.sec = 0;
                    
    goal_traj.goal.trajectory.points.push_back(point);
    
    return 1;   
}

void algorithmImplementation::initializeKinematics(){
    
    std::string configFile;
    std::string path = ros::package::getPath("schunk_lwa4p_kinematics");
    
    ros::NodeHandle private_node_handle_("~");

    //getting ros params
    private_node_handle_.param("config_file", configFile, path + std::string("/config/lwa4p_configuration.yaml"));
    
    kinematic.loadParameters(robot_id, configFile);
}

void algorithmImplementation::initializePosition(){
    
    // ROS needs some time to register the core and to establish all subscriber connections.
    // Since only one message is sent in the beginning, it is lost. Therefore, loop until connection
    // is established. 
    ros::Rate poll_rate(100);
    while(jointStatesPub.getNumSubscribers() == 0){
        poll_rate.sleep();
    }
    
    Eigen::MatrixXd goal_w = Eigen::MatrixXd::Zero(9, 1);
    Eigen::MatrixXd goal_q, q0;
    bool skip = false;
    
    goal_w(0,0) = x0;
    goal_w(1,0) = y0;
    goal_w(2,0) = z0;
//     goal_w(3,0) = 0;
//     goal_w(4,0) = 0;
//     goal_w(5,0) = 1;
//     goal_w(6,0) = 1;
//     goal_w(7,0) = 0;
//     goal_w(8,0) = 0;
    goal_w(3,0) = 0;
    goal_w(4,0) = 1;
    goal_w(5,0) = 0;
    goal_w(6,0) = -1;
    goal_w(7,0) = 0;
    goal_w(8,0) = 0;
    
    goal_q = kinematic.inverseKinematics(goal_w);
    //std::cout << goal_q << std::endl;
    
    q0 = Eigen::MatrixXd::Zero(6, 1);
    goal_q = kinematic.inverseKinematics_closestQ(goal_w, q0); // returns closest solution
    
    // check if nan appears in the solution, if it appears, ignore this result and repeat the procedure
    for (int i = 0; i < 6; i = i + 1){
        if (std::isnan(goal_q(i,0))) {
            std::cout << "Inverse!" << std::endl;
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
        
        bool finished = false;
        while(!finished){
            result_queue.callOne(ros::WallDuration());
            if (result == 3 && error == 0) finished = true;
        }
        result = -1;
        error = 1;
        std::cout << "In starting position!" << std::endl;
        ros::Duration(5).sleep();
        std::cout << "Ready!" << std::endl;
    }
}

void algorithmImplementation::initializeBuffer(){  
    // initialize buffer to hold past values of mutual gaze
    for (int i = 0; i < bufferSize; i = i + 1){
        buffer.push(0);
    }
    bufferSum = 0;
}

double algorithmImplementation::wrapToPi(double angle){

    return angle - floor(angle/(2*M_PI) + 0.5)*2*M_PI;
}

void algorithmImplementation::trackGaze(){
    
    if (std::abs(horGaze) < horGazeTolerance && std::abs(verGaze) < verGazeTolerance) {
        // false detection of non mutual gaze, mutual gaze is lost for a brief moment
        std::cout << "Lost you for a moment there!" << std::endl;
    }
    else {
        std::cout << "Move robot in gaze direction: " << horGaze << " " << verGaze << std::endl;
        bool skip = false;
        
        // proportional controller
        verGaze = 0.05 * verGaze;
        horGaze = 0.05 * horGaze;
        
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
        //std::cout << goal_q << std::endl;
            
        goal_q = kinematic.inverseKinematics_closestQ(goal_w, lwa4p_temp_q); // returns closest solution
        //std::cout << goal_q << std::endl;
        
        // check if nan appears in the solution, if it appears, ignore this result and repeat the procedure
        for (int i = 0; i < 6; i = i + 1){
            if (std::isnan(goal_q(i,0))) {
                skip = true;
                std::cout << "Inverse kinematics solution not feasible!" << std::endl;
                std::cout << "Check wanted position..." << std::endl;
                break;
            }
        }    
        
        if (jacobian && !skip){
            std::cout << "jacobijan" << std::endl;
            Eigen::MatrixXd jacob, jacob_inv, dq;
            Eigen::MatrixXd delta_w = Eigen::MatrixXd::Zero(9,1);
            Eigen::MatrixXd temp_goal_q = Eigen::MatrixXd::Zero(6,1);
            
            delta_w(0,0) = T(0,3) - T_06(0,3);
            delta_w(1,0) = T(1,3) - T_06(1,3);
            delta_w(2,0) = T(2,3) - T_06(2,3);
            delta_w(3,0) = T(0,0) - T_06(0,0);
            delta_w(4,0) = T(1,0) - T_06(1,0);
            delta_w(5,0) = T(2,0) - T_06(2,0);
            delta_w(6,0) = T(0,2) - T_06(0,2);
            delta_w(7,0) = T(1,2) - T_06(1,2);
            delta_w(8,0) = T(2,2) - T_06(2,2);
            
            jacob = calcJacobian(lwa4p_temp_q);
            jacob_inv = pseudoInverse(jacob);   
            
            dq = jacob_inv * delta_w;
            //TODO: See if wrapToPi is needed
    //         temp_goal_q(0, 0) = wrapToPi(lwa4p_temp_q(0, 0) + dq(0, 0));
    //         temp_goal_q(1, 0) = wrapToPi(lwa4p_temp_q(1, 0) + dq(1, 0));
    //         temp_goal_q(2, 0) = wrapToPi(lwa4p_temp_q(2, 0) - dq(2, 0));
    //         temp_goal_q(3, 0) = wrapToPi(lwa4p_temp_q(3, 0) + dq(3, 0));
    //         temp_goal_q(4, 0) = wrapToPi(lwa4p_temp_q(4, 0) + dq(4, 0));
    //         temp_goal_q(5, 0) = wrapToPi(lwa4p_temp_q(5, 0) + dq(5, 0));
            
            temp_goal_q(0, 0) = lwa4p_temp_q(0, 0) + dq(0, 0);
            temp_goal_q(1, 0) = lwa4p_temp_q(1, 0) + dq(1, 0);
            temp_goal_q(2, 0) = lwa4p_temp_q(2, 0) - dq(2, 0);
            temp_goal_q(3, 0) = lwa4p_temp_q(3, 0) + dq(3, 0);
            temp_goal_q(4, 0) = lwa4p_temp_q(4, 0) + dq(4, 0);
            temp_goal_q(5, 0) = lwa4p_temp_q(5, 0) + dq(5, 0);

            goal_q = temp_goal_q;
            
        }
        
//         for (int i = 0; i < 6; i = i + 1){
//             if (std::isnan(goal_q(i,0))) {
//                 skip = true;
//                 std::cout << "Inverse kinematics solution not feasible!" << std::endl;
//                 std::cout << "Check wanted position..." << std::endl;
//                 break;
//             }
//         }
        
        if (!skip) {
            T_start = ros::Time::now();
            goal_traj = {};
            prepareMsg(goal_q, T_start, goal_traj);
            
            jointStatesPub.publish(goal_traj);
            
            while(ros::ok()){
                result_queue.callOne(ros::WallDuration());
                if (result == 3 && error == 0) break;
            }
            result = -1;
            error = 1;
        }
    }
}

void algorithmImplementation::run(){
    
    T_start.sec = 0;
    T_start.nsec = 0;
    
    initializeKinematics();

    initializePosition();
    
    initializeBuffer();

    ros::Rate r(10); // rate is set to one because algorithm is tested much easier
    while(ros::ok()){
        
        ros::spinOnce();

        if (firstMutGazeDetected){
            if (mutGaze) {
                //waitGazeChange    
                std::cout << "Mutual gaze detected!" << std::endl;

            }
            else {
                trackGaze();
            }
        }
        
        // if first mutual gaze is still not detected, do not move robot
        else {
            std::cout << "Hold the mutual gaze!" << std::endl;
            if (bufferSum > threshold) {
                firstMutGazeDetected = true;
            }
        }
        r.sleep();
    }   
}