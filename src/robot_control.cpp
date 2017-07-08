#include <robot_control/pointMessages.h> 

int main (int argc, char **argv) {    
    
    ros::init(argc, argv, "robot_control");
    
    pointMessages tracker;
    
    tracker.run();
    
    return 0;
}