#pragma once
#include <color_cout.hpp> //lib
#include <fstream>
#include <iostream>
// #include <sys/type.h>
#include <patrolling_sim/MissionRequest.h>
#include <patrolling_sim/TaskRequest.h>
#include <ros/ros.h>
#include <ros/package.h> //to get pkg path
#include <std_msgs/Bool.h>
#include <std_msgs/Int16MultiArray.h>
#include <sys/stat.h>
#include <task_planner/Mission.h>
#include <task_planner/Task.h>
#include <algorithm>
#include <vector>

#include "getgraph.hpp"

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
};

inline bool operator<(const Task& A, const Task& B)
{
  if (!A.take && !B.take)
  {
    return A.dst < B.dst ? 1 : 0; 
  }
}

inline bool operator==(const Task& A, const Task& B)
{
    return A.dst == B.dst ? 1 : 0;
}

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

const std::string PS_path = ros::package::getPath("task_planner");

class TaskPlanner
{
public:
  TaskPlanner(ros::NodeHandle &nh_, uint TEAMSIZE);
  ~TaskPlanner(){};

  const char *task_file = "/home/dave/LogisticAgent_ws/src/task_planner/param/all_task.txt";

  std::string graph_file;
  std::string mapname;
  uint dimension;
  vertex *vertex_web;

  uint all_capacity = 0;

  uint TEAM_t;

  uint src_vertex = 6;
  uint dst_vertex[3] = {11, 16, 21};
  uint under_pass[7] = {7, 9, 12, 14, 17, 19, 22};
  uint upper_pass[7] = {5, 8, 10, 13, 15, 18, 20};
  vector<Task> tasks;
  vector<uint> route;
  bool *arrived_message;

  Task operator[](int i) const
  {
    return tasks[i];
  }
  Task &operator[](int i)
  {
    return tasks[i];
  }

  void t_print(Task t);
  void r_print();
  void t_generator();

  void compute_route_to_delivery(Task& t);
  void compute_route_to_picktask(Task& t); 
  int compute_cost_of_route(uint element);

  

  void init(int argc, char** argv);
  void run();

  Task compare(Task t1, Task t2);

  void task_Callback(const patrolling_sim::TaskRequestConstPtr &msg);
  // void mission_Callback(const patrolling_sim::MissionRequestConstPtr &msg);

private:
  ros::Subscriber sub_task; // quando un robot vuole un task
  ros::Publisher pub_task;  // pubblicazione dell'array (pop dal vettore di tasks)
  // ros::Subscriber sub_mission;
  // ros::Publisher pub_mission;
  // ros::Publisher pub_task_to_coo;
};

} // namespace taskplanner

#include "impl/TaskPlanner.i.hpp"