#include "TaskPlanner.hpp"

int main(int argc, char **argv)
{

    uint TEAMSIZE = atoi(argv[1]);

    ros::init(argc, argv, "task_planner");

    ros::NodeHandle nh_("~");

    taskplanner::TaskPlanner TP(nh_, TEAMSIZE);

    ros::AsyncSpinner spinner(2);

    spinner.start();

    ros::waitForShutdown();

    return 0;
}