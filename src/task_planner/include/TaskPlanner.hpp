#pragma once
#include <color_cout.hpp>  //lib
#include <fstream>
#include <iostream>
// #include <sys/type.h>
#include <patrolling_sim/MissionRequest.h>
#include <patrolling_sim/TaskRequest.h>
#include <ros/package.h>  //to get pkg path
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16MultiArray.h>
#include <sys/stat.h>
#include <task_planner/Mission.h>
#include <task_planner/Task.h>
#include <algorithm>
#include <vector>

#include "getgraph.hpp"

#define INIT_MSG 46

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

inline bool operator<(const Task &A, const Task &B)
{
  if (!A.take && !B.take)
  {
    return (A.dst < B.dst) ? 1 : 0;
  }
}

inline bool pop_min_element(const Task &A, const Task &B)
{
  if ((A.dst < B.dst) && (A.demand < B.demand))
  {
    return 1;
  }
  else if (A.dst == B.dst)
  {
      return 1;
  }
  return 0;
}

inline bool operator>(const Task &A, const Task &B)
{
  if (!A.take && !B.take)
  {
    return A.dst > B.dst ? 1 : 0;
  }
}

inline bool operator==(const Task &A, const Task &B)
{
  return A.order == B.order ? 1 : 0;
}

inline Task mkTask(int item, int order, int demand, int priority, int src, int dst, int edge)
{
  Task t;

  t.take = false;         //  flag
  t.item = item;          //  tipo di oggetto
  t.order = order;        //  id dell' ordine
  t.demand = demand;      //  quantita' di oggetti
  t.priority = priority;  //  priorita'
  t.src = src;
  t.dst = dst;
  t.edge = edge;

  return t;
}

struct ProcessAgent
{
  uint ID_ROBOT;
  uint CAPACITY;
  bool flag;
  vector<Task> mission; // task da concatenare
  vector<uint> route; // route finale concatenata
  vector<bool> status;
  int * total_item;
  uint   total_demand;
};

ostream &operator<<(ostream &os, const ProcessAgent &pa)
{
  os << "\nProcessAgent id: " << pa.ID_ROBOT << "\n";
  for (auto i = 0; i < pa.mission.size(); i++)
  {
    os << "- id_order: " << pa.mission[i].order << "\n"
       << "-   demand: " << pa.mission[i].demand << "\n"
       << "-      dst: " << pa.mission[i].dst << "\n"
       << "\n";
  }
  os << "\n";
}

inline ProcessAgent mkPA(uint id, uint c)
{
  ProcessAgent pa;

  pa.ID_ROBOT = id;
  pa.CAPACITY = c;
  pa.flag = false;
  pa.mission.clear();
  pa.route.clear(); // route definitiva
  pa.status.clear();
  pa.total_demand = 0;
  pa.total_item[3];

  return pa;
}

const std::string PS_path = ros::package::getPath("task_planner");

class TaskPlanner
{
public:
  TaskPlanner(ros::NodeHandle &nh_);
  ~TaskPlanner(){};

  const char *task_file = "/home/dave/LogisticAgent_ws/src/task_planner/param/all_task.txt";

  std::string graph_file;
  std::string mapname;
  uint dimension;
  vertex *vertex_web;

  uint TEAM_c = 0;
  uint TEAM_t = 0;
  uint nTask = 0;

  uint src_vertex = 6;
  uint dst_vertex[3] = { 11, 16, 21 };
  uint under_pass[7] = { 7, 9, 12, 14, 17, 19, 22 };
  uint upper_pass[7] = { 5, 8, 10, 13, 15, 18, 20 };
  uint initial_position[4] = { 2, 1, 0, 3 };
  vector<Task> tasks;
  vector<Task> skip_tasks;
  vector<Task> natblida;
  vector<uint> route;
  vector<bool> status;

  bool *init_agent;
  // vector<ProcessAgent> pa;
  ProcessAgent *pa;

  uint id = 0;

  Task operator[](int i) const
  {
    return tasks[i];
  }
  Task &operator[](int i)
  {
    return tasks[i];
  }

  void t_print(Task t);
  void pa_print(ProcessAgent pa);
  void t_generator();

  void compute_route_to_delivery(Task &t);
  void compute_route_to_picktask(Task &t);
  int compute_cost_of_route();

  void conclave(ProcessAgent &pa);

  void init(int argc, char **argv);
  void run();
  void task_Callback(const patrolling_sim::TaskRequestConstPtr &msg);
  void init_Callback(const std_msgs::Int16MultiArrayConstPtr &msg);
  void mission_Callback(const patrolling_sim::MissionRequestConstPtr &msg);

private:
  ros::Subscriber sub_task;  // quando un robot vuole un task
  ros::Subscriber sub_mission;
  
  ros::Subscriber sub_init;
  ros::Publisher pub_task;  // pubblicazione dell'array (pop dal vettore di tasks)
  ros::Publisher pub_results;
  // ros::Subscriber sub_mission;
  // ros::Publisher pub_mission;
  // ros::Publisher pub_task_to_coo;
};

}  // namespace taskplanner

#include "impl/TaskPlanner.i.hpp"