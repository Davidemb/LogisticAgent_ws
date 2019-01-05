#pragma once
#include <color_cout.hpp> //lib
#include <fstream>
#include <iostream>
// #include <sys/type.h>
#include <sys/stat.h>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16MultiArray.h>
#include <algorithm>
#include <patrolling_sim/TaskRequest.h>
#include <patrolling_sim/MissionRequest.h>
#include <task_planner/Task.h>
#include <task_planner/Mission.h>

namespace taskplanner
{
struct Task
{
    bool take;
    int item;
    int order;
    int demand;
    int priority;
    int src;
    int dst;
    int edge;

    bool operator < (const Task& other) const 
    {
        return dst < other.dst;
    }
};

inline Task mkTask(int item, int order, int demand, int priority, int src, int dst, int edge)
{
    Task t;

    t.take = false;        //  flag
    t.item = item;         //  tipo di oggetto
    t.order = order;       //  id dell' ordine
    t.demand = demand;     //  quantita' di oggetti
    t.priority = priority; //  priorita'
    t.src = src;
    t.dst = dst;
    t.edge = edge;

    return t;
}

class TaskPlanner
{

  public:

    TaskPlanner(ros::NodeHandle &nh_, uint TEAMSIZE);
    ~TaskPlanner(){};
    
    const char *task_file = "/home/dave/LogisticAgent_ws/src/task_planner/param/all_task.txt";

    uint all_capacity = 0;

    uint TEAM_t;

    uint src_vertex = 3;
    uint dst_vertex[4] = {6, 9, 12, 15};
    vector<Task> tasks;
    bool *arrived_message;

    Task operator[](int i) const { return tasks[i]; }
    Task &operator[](int i) { return tasks[i]; }
   
    void t_print(Task t);
    void t_generator();

    Task compare(Task t1, Task t2);
    
    void task_Callback(const patrolling_sim::TaskRequestConstPtr &msg);
    // void mission_Callback(const patrolling_sim::MissionRequestConstPtr &msg);

  private:
    ros::Subscriber sub_task; // quando un robot vuole un task
    ros::Publisher  pub_task; // pubblicazione dell'array (pop dal vettore di tasks)
    ros::Subscriber sub_mission;
    ros::Publisher  pub_mission;
    ros::Publisher  pub_task_to_coo;
};

} // namespace taskplanner

#include "impl/TaskPlanner.i.hpp"