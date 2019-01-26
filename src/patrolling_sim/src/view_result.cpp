#include <float.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <string>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/package.h> //to get pkg path
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <getgraph.hpp>
#include "message_types.hpp"
// #include "patrolling_sim/GoToStartPosSrv.h"
#include <color_cout.hpp>

using namespace std;

using uint = unsigned int;

struct view_result
{
  uint ID_ROBOT;
  uint n_task;
  uint type_A; // demand dell'item per tipo di oggeto
  uint type_B;
  uint type_C;
  int dim_path;
  uint interference;
  uint resend_goal;
};

ros::Subscriber results_sub;
ros::Publisher results_pub;

// Initialization:
bool initialize = true; // Initialization flag
uint cnt = 0;           // Count number of robots connected
uint teamsize;
bool init_robots[NUM_MAX_ROBOTS];
double last_goal_reached[NUM_MAX_ROBOTS];

vector<view_result> view_results(4);

view_result *vrs;

// mutex for accessing last_goal_reached vector
pthread_mutex_t lock_last_goal_reached;

// State Variables:
bool goal_reached = false;

int goal;
double time_zero, last_report_time;
time_t real_time_zero;
double goal_reached_wait, comm_delay, lost_message_rate;
string algorithm, algparams, nav_mod, initial_positions;

const std::string PS_path = ros::package::getPath("patrolling_sim"); // D.Portugal => get pkg path

int number_of_visits[MAX_DIMENSION];
size_t dimension; // graph size

uint interference_cnt = 0;
uint resendgoal_cnt = 0;
uint patrol_cnt = 1;
void set_last_goal_reached(int id, int k)
{
  pthread_mutex_lock(&lock_last_goal_reached);
  c_print("hole", green);
  pthread_mutex_unlock(&lock_last_goal_reached);
}

void update_stats(int id_robot, int goal)
{
  //   printf("last_visit [%d] = %.1f\n", goal, last_visit [goal]);
  double current_time = ros::Time::now().toSec();

  printf("Robot %d reached goal %d (current time: %.2f, alg: %s, nav: %s)\n", id_robot, goal, current_time,
         algorithm.c_str(), nav_mod.c_str());

  double last_visit_temp = current_time - time_zero; // guarda o valor corrente
  number_of_visits[goal]++;

  set_last_goal_reached(id_robot, current_time);

  printf("   nr_of_visits = %d -", number_of_visits[goal]);

  goal_reached = false;
}

double get_last_goal_reached(int k)
{
  pthread_mutex_lock(&lock_last_goal_reached);
  double r = last_goal_reached[k];
  pthread_mutex_unlock(&lock_last_goal_reached);
  return r;
}

void resultsCB(const std_msgs::Int16MultiArray::ConstPtr &msg)
{
  std::vector<signed short>::const_iterator it = msg->data.begin();

  std::vector<int> vresults;

  view_result vr;

  vresults.clear();

  for (size_t k = 0; k < msg->data.size(); k++)
  {
    vresults.push_back(*it);
    it++;
  }

  int id_robot = vresults[0]; // robot sending the message
  int msg_type = vresults[1]; // message type

  switch (msg_type)
  {
  case INITIALIZE_MSG_TYPE:
  {
    if (initialize && vresults[2] == 1)
    {
      if (init_robots[id_robot] == false)
      { // receive init msg: "ID,msg_type,1"
        printf("Robot [ID = %d] is Active!\n", id_robot);
        init_robots[id_robot] = true;

        // Patch D.Portugal (needed to support other simulators besides Stage):
        double current_time = ros::Time::now().toSec();
        // initialize last_goal_reached:
        set_last_goal_reached(id_robot, current_time);

        cnt++;
      }

      printf("All Robots GO!\n");
      initialize = false;

      // Clock Reset:
      time_zero = ros::Time::now().toSec();
      last_report_time = time_zero;

      time(&real_time_zero);
      printf("Time zero = %.1f (sim) = %lu (real) \n", time_zero, (long)real_time_zero);

      std_msgs::Int16MultiArray msg; // -1,msg_type,100,0,0
      msg.data.clear();
      msg.data.push_back(-1);
      msg.data.push_back(INITIALIZE_MSG_TYPE);
      msg.data.push_back(100); // Go !!!
      results_pub.publish(msg);
      ros::spinOnce();
    }
  }
  break;

  case TARGET_REACHED_MSG_TYPE:
  {
    // goal sent by a robot during the experiment
    // [ID,msg_type,vertex,intention,0]
    if (initialize == false)
    {
      goal = vresults[2];
      ROS_INFO("Robot %d reached Goal %d.\n", id_robot, goal);
      fflush(stdout);
      goal_reached = true;
      update_stats(id_robot, goal);
      ros::spinOnce();
    }
    break;
  }

  case INTERFERENCE_MSG_TYPE:
  {
    // interference: [ID,msg_type]
    if (initialize == false)
    {
      ROS_INFO("Robot %d sent interference.\n", id_robot);
      // interference_cnt++;
      view_results.at(id_robot).interference += 1;
      // view_results.at(id_robot).resend_goal += vresults[6];
      ros::spinOnce();
    }
    break;
  }

  case RESENDGOAL_MSG_TYPE:
  {
    if (initialize == false)
    {
      ROS_INFO("Robot %d sent resendgoal.\n", id_robot);
      // resendgoal_cnt++;
      // view_results.at(id_robot).interference += vresults[5];
      view_results.at(id_robot).resend_goal += 1;
      ros::spinOnce();
    }
    break;
  }

  case TASK_REACHED_MSG_TYPE:
  {
    if (initialize == false)
    {
      view_results.at(id_robot).ID_ROBOT = id_robot;
      view_results.at(id_robot).n_task += vresults[2]; // capacita' demand del task
      switch (vresults[3])
      {
      case 0:
      {
        view_results.at(id_robot).type_A += 1;
      }
      break;
      case 1:
      {
        view_results.at(id_robot).type_B += 1;
      }
      break;
      case 2:
      {
        view_results.at(id_robot).type_C += 1;
      }
      break;
      }
      view_results.at(id_robot).dim_path += vresults[4];
    }

    c_print("\nview result?", red);
    for (int i = 0; i < view_results.size(); i++)
    {
      cout << view_results[i].ID_ROBOT << " "
           << view_results[i].n_task << " "
           << view_results[i].type_A << " "
           << view_results[i].type_B << " "
           << view_results[i].type_C << " "
           << view_results[i].dim_path << " "
           << view_results[i].interference << " "
           << view_results[i].resend_goal << "\n";
    }
    cout << "\n";
  }
  }
}

bool check_dead_robots()
{
  sleep(300);
  return true;
}

void scenario_name(char *name, const char *graph_file, const char *teamsize_str)
{
  uint i, start_char = 0, end_char = strlen(graph_file) - 1;

  for (i = 0; i < strlen(graph_file); i++)
  {
    if (graph_file[i] == '/' && i < strlen(graph_file) - 1)
    {
      start_char = i + 1;
    }

    if (graph_file[i] == '.' && i > 0)
    {
      end_char = i - 1;
      break;
    }
  }

  for (i = start_char; i <= end_char; i++)
  {
    name[i - start_char] = graph_file[i];
    if (i == end_char)
    {
      name[i - start_char + 1] = '\0';
    }
  }

  strcat(name, "_");
  strcat(name, teamsize_str);
}

void finish_simulation()
{ //-1,msg_type,999,0,0
  ROS_INFO("Sending stop signal to patrol agents.");
  std_msgs::Int16MultiArray msg;
  msg.data.clear();
  msg.data.push_back(-1);
  msg.data.push_back(INITIALIZE_MSG_TYPE);
  msg.data.push_back(999); // end of the simulation
  results_pub.publish(msg);
  ros::spinOnce();
}

int main(int argc, char **argv)
{ // pass TEAMSIZE GRAPH ALGORITHM
  /*
 argc=3
 argv[0]=/.../patrolling_sim/bin/monitor
 argv[1]=grid
 argv[2]=ALGORITHM = {MSP,Cyc,CC,CR,HCR}
 argv[3]=TEAMSIZE
 */

  // ex: "rosrun patrolling_sim monitor maps/example/example.graph MSP 2"

  //   uint teamsize;
  char teamsize_str[3];
  teamsize = atoi(argv[3]);

  if (teamsize >= NUM_MAX_ROBOTS || teamsize < 1)
  {
    ROS_INFO("The Teamsize must be an integer number between 1 and %d", NUM_MAX_ROBOTS);
    return 0;
  }
  else
  {
    strcpy(teamsize_str, argv[3]);
    //     printf("teamsize: %s\n", teamsize_str);
    //     printf("teamsize: %u\n", teamsize);
  }

  algorithm = string(argv[2]);
  printf("Algorithm: %s\n", algorithm.c_str());

  string mapname = string(argv[1]);
  string graph_file = "maps/" + mapname + "/" + mapname + ".graph";

  printf("Graph: %s\n", graph_file.c_str());

  /** D.Portugal: needed in case you "rosrun" from another folder **/
  chdir(PS_path.c_str());

  // Check Graph Dimension:
  dimension = GetGraphDimension(graph_file.c_str());
  if (dimension > MAX_DIMENSION)
  {
    cout << "ERROR!!! dimension > MAX_DIMENSION (static value) !!!" << endl;
    abort();
  }
  printf("Dimension: %u\n", (uint)dimension);

  char hostname[80];

  int r = gethostname(hostname, 80);
  if (r < 0)
    strcpy(hostname, "default");

  printf("Host name: %s\n", hostname);

  for (size_t i = 0; i < NUM_MAX_ROBOTS; i++)
  {
    init_robots[i] = false;
    last_goal_reached[i] = 0.0;
  }

  bool dead = false; // check if there is a dead robot

  bool simrun, simabort; // check if simulation is running and if it has been
                         // aborted by the user

  // Scenario name (to be used in file and directory names)
  char sname[80];
  scenario_name(sname, graph_file.c_str(), teamsize_str);

  // Create directory results if does not exist
  string path1 = "results";

  string path2, path3, path4;

  path2 = path1 + "/" + string(sname);
  path3 = path2 + "/" + algorithm;
  path4 = path3 + "/" + hostname;

  struct stat st;

  if (stat(path1.c_str(), &st) != 0)
    mkdir(path1.c_str(), 0777);
  if (stat(path2.c_str(), &st) != 0)
    mkdir(path2.c_str(), 0777);
  if (stat(path3.c_str(), &st) != 0)
    mkdir(path3.c_str(), 0777);
  if (stat(path4.c_str(), &st) != 0)
    mkdir(path4.c_str(), 0777);

  printf("Path experimental results: %s\n", path4.c_str());

  // Local time (real clock time)
  time_t rawtime;
  struct tm *timeinfo;
  char strnow[80];

  time(&rawtime);
  timeinfo = localtime(&rawtime);
  sprintf(strnow, "%d%02d%02d_%02d%02d%02d", timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
          timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
  printf("Date-time of the experiment: %s\n", strnow);

  // File to log all the idlenesses of an experimental scenario

  string resultsfilename, resultstimecsvfilename, expname;
  expname = path4 + "/" + string(strnow);
  resultsfilename = expname + "_results.txt";
  resultstimecsvfilename = expname + "_timeresults.csv";

  FILE *fexplist;
  fexplist = fopen("experiments.txt", "a");
  fprintf(fexplist, "%s\n", expname.c_str());
  fclose(fexplist);

  FILE *resultstimecsvfile;
  resultstimecsvfile = fopen(resultstimecsvfilename.c_str(), "w");

  fprintf(resultstimecsvfile, "Time;Interferences\n"); // header

  // Wait for all robots to connect! (Exchange msgs)
  ros::init(argc, argv, "monitor");
  ros::NodeHandle nh;

  // Subscribe "results" from robots
  results_sub = nh.subscribe("results", 100, resultsCB);

  // Publish data to "results"
  results_pub = nh.advertise<std_msgs::Int16MultiArray>("results", 100);

  double duration = 0.0, real_duration = 0.0;

  ros::Rate loop_rate(30); // 0.033 seconds or 30Hz

  nh.setParam("/simulation_running", "true");
  nh.setParam("/simulation_abort", "false");

  double current_time = ros::Time::now().toSec();

  // read parameters
  if (!ros::param::get("/goal_reached_wait", goal_reached_wait))
  {
    goal_reached_wait = 0.0;
    ROS_WARN("Cannot read parameter /goal_reached_wait. Using default value 0.0!");
  }

  if (!ros::param::get("/communication_delay", comm_delay))
  {
    comm_delay = 0.0;
    ROS_WARN("Cannot read parameter /communication_delay. Using default value 0.0!");
  }

  if (!ros::param::get("/lost_message_rate", lost_message_rate))
  {
    lost_message_rate = 0.0;
    ROS_WARN("Cannot read parameter /lost_message_rate. Using default value 0.0!");
  }

  if (!ros::param::get("/initial_positions", initial_positions))
  {
    initial_positions = "default";
    ROS_WARN("Cannot read parameter /initial_positions. Using default value '%s'!", initial_positions.c_str());
  }

  if (!ros::param::get("/navigation_module", nav_mod))
  {
    ROS_WARN("Cannot read parameter /navigation_module. Using default value 'ros'!");
    nav_mod = "ros";
  }

  // mutex for accessing last_goal_reached vector
  pthread_mutex_init(&lock_last_goal_reached, NULL);

  while (ros::ok())
  {
    if (!initialize)
    { // check if msg is goal or interference -> compute
      // necessary results.

      // check time
      double report_time = ros::Time::now().toSec();

      // printf("### report time=%.1f  last_report_time=%.1f diff =
      // %.1f\n",report_time, last_report_time, report_time - last_report_time);

      // write results every TIMEOUT_WRITE_RESULTS_(FOREVER) seconds anyway
      bool timeout_write_results;

      timeout_write_results = (report_time - last_report_time > TIMEOUT_WRITE_RESULTS);

      if (timeout_write_results)
      {
        // write results every time a patrolling cycle is finished.
        // or after some time

        uint i, tot_visits = 0;
        for (size_t i = 0; i < dimension; i++)
        {
          tot_visits += number_of_visits[i];
        }
        float avg_visits = (float)tot_visits / dimension;

        duration = report_time - time_zero;
        time_t real_now;
        time(&real_now);
        real_duration = (double)real_now - (double)real_time_zero;

        if (timeout_write_results)
          last_report_time = report_time;
        else
          patrol_cnt++;

      } // if ((patrol_cnt == complete_patrol) || timeout_write_results)

// Check if simulation must be terminated
#if SIMULATE_FOREVER == false
      dead = check_dead_robots();

      simrun = true;
      simabort = false;
      std::string psimrun, psimabort;
      bool bsimabort;
      if (nh.getParam("/simulation_running", psimrun))
        if (psimrun == "false")
          simrun = false;
      if (nh.getParam("/simulation_abort", psimabort))
        if (psimabort == "true")
          simabort = true;
      if (nh.getParam("/simulation_abort", bsimabort))
        simabort = bsimabort;

      if ((dead) || (!simrun) || (simabort))
      {
        printf("Simulation is Over\n");
        nh.setParam("/simulation_running", false);
        finish_simulation();
        ros::spinOnce();
        break;
      }
#endif

    } // if ! initialize

    current_time = ros::Time::now().toSec();
    ros::spinOnce();
    loop_rate.sleep();

  } // while ros ok

  ros::shutdown();

  duration = current_time - time_zero;
  time_t real_now;
  time(&real_now);
  real_duration = (double)real_now - (double)real_time_zero;

  uint tot_visits = 0;
  for (size_t i = 0; i < dimension; i++)
  {
    tot_visits += number_of_visits[i];
  }
  float avg_visits = (float)tot_visits / dimension;

  // Write info file with overall results
  string infofilename;
  infofilename = expname + "_info.csv";

  FILE *infofile;
  infofile = fopen(infofilename.c_str(), "w");
  fprintf(infofile, "%s;%s;%s;%.1f;%.2f;%s;%s;%s;%s;%s;%.1f;%.1f;%d;%s;%.1f;%."
                    "1f;%.1f;%.1f;%.2f;%d;%.1f;%d\n",
          mapname.c_str(), teamsize_str, initial_positions.c_str(), goal_reached_wait, comm_delay, nav_mod.c_str(),
          algorithm.c_str(), algparams.c_str(), hostname, strnow, duration, real_duration, interference_cnt,
          (dead ? "FAIL" : (simabort ? "ABORT" : "TIMEOUT")),
          (float)interference_cnt / duration * 60, tot_visits, avg_visits);

  fclose(infofile);
  cout << "Info file " << infofilename << " saved." << endl;

  printf("Monitor closed.\n");
}