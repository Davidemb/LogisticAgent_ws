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
#include <ros/package.h>  //to get pkg path
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

using std::cout;
using std::endl;
using std::string;

using uint = unsigned int;

ros::Subscriber results_sub;
ros::Publisher results_pub, screenshot_pub;
ros::ServiceServer GotoStartPosMethod;

// Initialization:
bool initialize = true;       // Initialization flag
bool goto_start_pos = false;  // default: robots already start in right position
uint cnt = 0;                 // Count number of robots connected
uint teamsize;
bool init_robots[NUM_MAX_ROBOTS];
double last_goal_reached[NUM_MAX_ROBOTS];

// mutex for accessing last_goal_reached vector
pthread_mutex_t lock_last_goal_reached;

// State Variables:
bool goal_reached = false;

int goal;
double time_zero, last_report_time;
time_t real_time_zero;
double goal_reached_wait, comm_delay, lost_message_rate;
string algorithm, algparams, nav_mod, initial_positions;

const std::string PS_path = ros::package::getPath("patrolling_sim");  // D.Portugal => get pkg path


int number_of_visits[MAX_DIMENSION];
size_t dimension;  // graph size

uint interference_cnt = 0;
uint patrol_cnt = 1;

void update_stats(int id_robot, int goal);

double get_last_goal_reached(int k)
{
  pthread_mutex_lock(&lock_last_goal_reached);
  double r = last_goal_reached[k];
  pthread_mutex_unlock(&lock_last_goal_reached);
  return r;
}

void set_last_goal_reached(int k, double val)
{
  pthread_mutex_lock(&lock_last_goal_reached);
  last_goal_reached[k] = val;
  pthread_mutex_unlock(&lock_last_goal_reached);
}

void resultsCB(const std_msgs::Int16MultiArray::ConstPtr &msg)
{
  dolog("resultsCB - begin");

  std::vector<signed short>::const_iterator it = msg->data.begin();

  std::vector<int> vresults;

  vresults.clear();

  for (size_t k = 0; k < msg->data.size(); k++)
  {
    vresults.push_back(*it);
    it++;
  }

  int id_robot = vresults[0];  // robot sending the message
  int msg_type = vresults[1];  // message type

  switch (msg_type)
  {
    case INITIALIZE_MSG_TYPE:
    {
      if (initialize && vresults[2] == 1)
      {
        if (init_robots[id_robot] == false)
        {  // receive init msg: "ID,msg_type,1"
          printf("Robot [ID = %d] is Active!\n", id_robot);
          init_robots[id_robot] = true;

          // Patch D.Portugal (needed to support other simulators besides Stage):
          double current_time = ros::Time::now().toSec();
          // initialize last_goal_reached:
          set_last_goal_reached(id_robot, current_time);

          cnt++;
        }
        if (cnt == teamsize)
        {
          // check if robots need to travel to starting positions
          while (goto_start_pos)
          {  // if or while (?)

            // patrolling_sim::GoToStartPosSrv::Request Req;
            // Req.teamsize.data = teamsize;
            // Req.sleep_between_goals.data = 20; // time in secs to wait before
            // sending goals to each different
            // robot
            // patrolling_sim::GoToStartPosSrv::Response Rep;

            ROS_INFO("Sending all robots to starting position.");

            //  if (!ros::service::call("/GotoStartPosSrv", Req, Rep))
          /*   if( false)
            {  // blocking call
              ROS_ERROR("Error invoking /GotoStartPosSrv.");
              ROS_ERROR("Sending robots to initial position failed.");
              ros::shutdown();  // make sense for while implementation
              return;
            }
            else
            {
              goto_start_pos = false;
              system("rosnode kill GoToStartPos &");  // we don't need the service
                                                      anymore.
            } */
          }

          printf("All Robots GO!\n");
          initialize = false;

          // Clock Reset:
          time_zero = ros::Time::now().toSec();
          last_report_time = time_zero;

          time(&real_time_zero);
          printf("Time zero = %.1f (sim) = %lu (real) \n", time_zero, (long)real_time_zero);

          std_msgs::Int16MultiArray msg;  // -1,msg_type,100,0,0
          msg.data.clear();
          msg.data.push_back(-1);
          msg.data.push_back(INITIALIZE_MSG_TYPE);
          msg.data.push_back(100);  // Go !!!
          results_pub.publish(msg);
          ros::spinOnce();
        }
      }

      //}
      break;
    }

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
        interference_cnt++;
        ros::spinOnce();
      }
      break;
    }
  }

  dolog("resultsCB - end");
}

bool check_dead_robots()
{
  dolog("  check_dead_robots - begin");

  double current_time = ros::Time::now().toSec();
  bool r = false;
  for (size_t i = 0; i < teamsize; i++)
  {
    double l = get_last_goal_reached(i);
    double delta = current_time - l;
    // printf("DEBUG dead robot: %d   %.1f - %.1f =
    // %.1f\n",i,current_time,l,delta);
    if (delta > DEAD_ROBOT_TIME * 0.75)
    {
      printf("Robot %lu: dead robot - delta = %.1f / %.1f \n", i, delta, DEAD_ROBOT_TIME);
      system("play -q beep.wav");
    }
    if (delta > DEAD_ROBOT_TIME)
    {
      // printf("Dead robot %d. Time from last goal reached = %.1f\n",i,delta);
      r = true;
      break;
    }
  }

  dolog("  check_dead_robots - end");

  return r;
}

void update_stats(int id_robot, int goal)
{
  dolog("  update_stats - begin");

  //   printf("last_visit [%d] = %.1f\n", goal, last_visit [goal]);
  double current_time = ros::Time::now().toSec();

  printf("Robot %d reached goal %d (current time: %.2f, alg: %s, nav: %s)\n", id_robot, goal, current_time,
         algorithm.c_str(), nav_mod.c_str());

  double last_visit_temp = current_time - time_zero;  // guarda o valor corrente
  number_of_visits[goal]++;

  set_last_goal_reached(id_robot, current_time);

  printf("   nr_of_visits = %d -", number_of_visits[goal]);

  if (number_of_visits[goal] == 0)
  {
    avg_idleness[goal] = 0.0;
    stddev_idleness[goal] = 0.0;
    total_0[goal] = 0.0;
    total_1[goal] = 0.0;
    total_2[goal] = 0.0;
  }
  else
  {  // if (number_of_visits [goal] > 0) {

    current_idleness[goal] = last_visit_temp - last_visit[goal];

    if (current_idleness[goal] > max_idleness)
      max_idleness = current_idleness[goal];
    if (current_idleness[goal] < min_idleness || min_idleness < 0.1)
      min_idleness = current_idleness[goal];

    // global stats
    gT0++;
    gT1 += current_idleness[goal];
    gT2 += current_idleness[goal] * current_idleness[goal];

    // node stats
    total_0[goal] += 1.0;
    total_1[goal] += current_idleness[goal];
    total_2[goal] += current_idleness[goal] * current_idleness[goal];
    avg_idleness[goal] = total_1[goal] / total_0[goal];
    stddev_idleness[goal] = 1.0 / total_0[goal] * sqrt(total_0[goal] * total_2[goal] - total_1[goal] * total_1[goal]);

    printf(" idl current = %.2f, ", current_idleness[goal]);
    printf(" avg = %.1f, stddev = %.1f,", avg_idleness[goal], stddev_idleness[goal]);
    printf(" max = %.1f - interf = %d\n", max_idleness, interference_cnt);

    // save data in idleness file
    fprintf(idlfile, "%.1f;%d;%d;%.1f;%d\n", current_time, id_robot, goal, current_idleness[goal], interference_cnt);
    fflush(idlfile);

#if SAVE_HYSTOGRAMS
    // compute values for hystograms
    int b = (int)(current_idleness[goal] / RESOLUTION);
    if (b < hn)
    {
      hv[b]++;
      hsum++;
    }
#endif
  }

  complete_patrol = calculate_patrol_cycle(number_of_visits, dimension);
  printf("   complete patrol cycles = %d\n", complete_patrol);

  // Compute node with highest current idleness
  size_t hnode;
  double hidl = 0;
  for (size_t i = 0; i < dimension; i++)
  {
    double cidl = last_visit_temp - last_visit[i];
    if (cidl > hidl)
    {
      hidl = cidl;
      hnode = i;
    }
  }
  printf("   highest current idleness: node %lu idl %.1f\n\n", hnode, hidl);

  last_visit[goal] = last_visit_temp;

  goal_reached = false;

  dolog("  update_stats - end");
}

