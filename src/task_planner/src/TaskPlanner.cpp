#include "TaskPlanner.hpp"

int main(int argc, char **argv)
{

    uint TEAMSIZE = atoi(argv[2]);

    ros::init(argc, argv, "task_planner");

    ros::NodeHandle nh_("~");

    taskplanner::TaskPlanner TP(nh_);

    TP.init(argc, argv);

    c_print("inizializzazione finita!",green);

    // TP.run();
    
    ros::AsyncSpinner spinner(2);

    spinner.start();

    ros::waitForShutdown();

    return 0;
}