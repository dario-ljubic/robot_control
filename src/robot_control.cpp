#include <robot_control/algorithmImplementation.h> 

int main (int argc, char **argv) {    
    
    ros::init(argc, argv, "robot_control");
    
    algorithmImplementation tracker;
    
    tracker.run();
    
    return 0;
}