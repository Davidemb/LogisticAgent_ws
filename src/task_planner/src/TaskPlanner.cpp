#include "TaskPlanner.hpp"

int main(int argc,char** argv)
{
    ros::init(argc, argv, "task_planner");
    
    ros::NodeHandle nh_("~");
    
    taskplanner::TaskPlanner TP(nh_);
    
    ros::AsyncSpinner spinner(2);
    
    spinner.start();
    
    ros::waitForShutdown();
    
    return 0;
}