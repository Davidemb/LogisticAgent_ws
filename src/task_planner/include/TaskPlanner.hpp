#pragma once
#include <color_cout.hpp>  //lib
// #include <partition.hpp>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <iterator>
#include <set>
#include <vector>
#include <queue>
#include <sstream>

#include <ros/package.h>  //to get pkg path
#include <ros/ros.h>
#include <sys/stat.h>

#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Bool.h>
#include <patrolling_sim/TaskRequest.h>
#include <patrolling_sim/MissionRequest.h>
#include <task_planner/Mission.h>
#include <task_planner/Task.h>


#include "getgraph.hpp"
#include "partition.hpp"

#define INIT_MSG 46

namespace taskplanner
{
struct Task
{
  bool take;
  int item;
  int order;
  int demand;
  int dst;
};

inline bool operator<(const Task &A, const Task &B)
{
  // messo <= per insert nel task_set.insert()
  // if (!A.take && !B.take)
  // {
  return (A.dst <= B.dst) ? 1 : 0;
  // }
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

inline Task mkTask(int item, int order, int demand, int dst)
{
  Task t;

  t.take = false;     //  flag
  t.item = item;      //  tipo di oggetto
  t.order = order;    //  id dell' ordine
  t.demand = demand;  //  quantita' di oggetti
  t.dst = dst;

  return t;
}

ostream &operator<<(ostream &os, const Task &t)
{
  os << "t: " << t.order;
}

struct Route
{
  bool status;
  uint id_vertex;
};

struct ProcessAgent
{
  uint ID_ROBOT;
  uint CAPACITY;
  bool flag;
  vector<Task> mission;  // task da concatenare
  vector<Route> route;   // vettore di vertici del path finale
  uint *dst;
  uint *total_item;
  uint total_demand;
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
  for (auto k = 0; k < pa.route.size(); k++)
  {
    os << "- route: " << pa.route[k].id_vertex << " [" << pa.route[k].status << "] "
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
  pa.route.clear();  // route definitiva
  pa.dst;
  pa.total_demand = 0;
  pa.total_item;

  return pa;
}

inline bool operator>(const ProcessAgent &A, const ProcessAgent &B)
{
  return A.CAPACITY > B.CAPACITY ? 1 : 0;
}

inline bool operator<(const ProcessAgent &A, const ProcessAgent &B)
{
  return A.CAPACITY < B.CAPACITY ? 1 : 0;
}

struct ProcessTask
{
  uint id;
  uint tot_demand;
  uint path_distance;
  vector<Task> mission;
  vector<uint> route; 
};

inline bool operator==(const ProcessTask &A, const ProcessTask &B)
{
  return A.id == B.id ? 1 : 0;
}

inline bool operator<(const ProcessTask &A, const ProcessTask &B)
{
  return A.path_distance/A.tot_demand < B.path_distance/B.tot_demand ? 1 : 0;
}

ostream &operator<<(ostream &os, const ProcessTask &t)
{
  os << "ProcessTask: " << "\n"
     << " -       id: " << t.id << "\n"
     << " -   demand:" <<t.tot_demand << "\n"
     << "\n";
}

struct CandidateTask
{
  uint id;
  uint subset;
  vector<vector<Task>> vv_t;
};

inline CandidateTask mkCT(int id, int subset)
{
  CandidateTask ct;

  ct.id = id;
  ct.subset = subset;
  ct.vv_t.resize(subset);

  return ct; 
}

const std::string PS_path = ros::package::getPath("task_planner");

class TaskPlanner
{
public:
  TaskPlanner(ros::NodeHandle &nh_);
  ~TaskPlanner(){};

  vertex *vertex_web;

  uint TEAM_c = 0;
  uint TEAM_t = 0;
  uint nTask = 0;
  uint id = 0;
  uint src_vertex = 6;
  uint dst_vertex[3] = { 11, 16, 21 };
  uint under_pass[7] = { 7, 9, 12, 14, 17, 19, 22 };
  uint upper_pass[7] = { 5, 8, 10, 13, 15, 18, 20 };
  uint initial_position[4] = { 2, 1, 0, 3 };
  // per ora statici senza funzioni
  uint p_11[8] = { 6, 7, 9, 12, 11, 10, 8, 5 };
  uint p_16[12] = { 6, 7, 9, 12, 14, 17, 16, 15, 13, 10, 8, 5 };
  uint p_21[16] = { 6, 7, 9, 12, 14, 17, 19, 22, 21, 20, 18, 15, 13, 10, 8, 5 };
  
  uint p_11_16[14] = { 6, 7, 9, 12, 11, 12, 14, 17, 16, 15, 13, 10, 8, 5 };
  uint p_11_21[18] = { 6, 7, 9, 12, 11, 12, 14, 17, 19, 22, 21, 20, 18, 15, 13, 10, 8, 5 };
  uint p_16_21[18] = { 6, 7, 9, 12, 14, 17, 19, 22, 21, 20, 18, 15, 13, 10, 8, 5 };

  uint p_11_16_21[20] = { 6, 7, 9, 12, 11, 12, 14, 17, 16, 17, 19, 22, 21, 20, 18, 15, 13, 10, 8, 5 };
  //----------------------------------------------------------------------------

  vector<Task> tasks;
  vector<Task> skip_tasks;
  vector<Task> natblida;  // tutte le combinazioni di task per il conclave()
  set<Task> task_set;

  bool *init_agent;
  ProcessAgent *pa;

  // bool finish_task = true;


  vector<ProcessTask> v_pt;
  vector<CandidateTask> v_ct;
  priority_queue<ProcessTask> p_q;

  Task operator[](int i) const
  {
    return tasks[i];
  }
  Task &operator[](int i)
  {
    return tasks[i];
  }
  
  // vector<vector<>> v_v_T;

  void t_print(Task &t);
  void pa_print(ProcessAgent &pa);
  void ct_print(CandidateTask &ct);
  void t_generator();

  void compute_route_to_delivery(ProcessAgent *pa);
  void compute_route_to_picktask(ProcessAgent *pa);
  int compute_cost_of_route(ProcessAgent *pa);
  void compute_opt_delivery();

  int ccor(vector<uint> route);
  uint compute_cycle_dst(vector<Task> mission);

  void conclave(ProcessAgent &pa);
  set<set<Task>> powerSet(const set<Task> &t);

  vector<Task> processedTask();
  void prepare_missions();
  void compute_best_subtask();

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
};

}  // namespace taskplanner

#include "impl/TaskPlanner.i.hpp"