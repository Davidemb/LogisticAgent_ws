#pragma once

namespace patrolagent
{
using namespace std;

void PatrolAgent::init(int argc, char **argv)
{
  /*
      argv[0]=/.../patrolling_sim/bin/GBS
      argv[1]=__name:=XXXXXX
      argv[2]=grid
      argv[3]=ID_ROBOT
  */

  srand(time(NULL));

  // More than One robot (ID between 0 and 99)
  if (atoi(argv[3]) > NUM_MAX_ROBOTS || atoi(argv[3]) < -1)
  {
    ROS_INFO("The Robot's ID must be an integer number between 0 an 99"); // max 100
                                                                          // robots
    return;
  }
  else
  {
    ID_ROBOT = atoi(argv[3]);
  }

  /** D.Portugal: needed in case you "rosrun" from another folder **/
  chdir(PS_path.c_str());

  mapname = string(argv[2]);
  graph_file = "maps/" + mapname + "/" + mapname + ".graph";

  // Check Graph Dimension:
  dimension = GetGraphDimension(graph_file.c_str());

  // Create Structure to save the Graph Info;
  vertex_web = new vertex[dimension];

  // Get the Graph info from the Graph File
  GetGraphInfo(vertex_web, dimension, graph_file.c_str());

  uint nedges = GetNumberEdges(vertex_web, dimension);

  printf("Loaded graph %s with %d nodes and %d edges\n", mapname.c_str(), dimension, nedges);

#if 0
    /* Output Graph Data */   
    for (int i=0;i<dimension;i++){
        printf ("ID= %u\n", vertex_web[i].id);
        printf ("X= %f, Y= %f\n", vertex_web[i].x, vertex_web[i].y);
        printf ("#Neigh= %u\n", vertex_web[i].num_neigh);
        
        for (int j=0;j<vertex_web[i].num_neigh; j++){
        printf("\tID = %u, DIR = %s, COST = %u\n", vertex_web[i].id_neigh[j], vertex_web[i].dir[j], vertex_web[i].cost[j]);
        }
        
        printf("\n");   
    }
#endif

  interference = false;
  ResendGoal = false;
  goal_complete = true; // inizializzazione della Flag
  last_interference = 0;
  goal_canceled_by_user = false;
  aborted_count = 0;
  resend_goal_count = 0;
  communication_delay = 0.0;
  lost_message_rate = 0.0;
  goal_reached_wait = 0.0;

  /* Define Starting Vertex/Position (Launch File Parameters) */
  ros::init(argc, argv, "patrol_robot"); // will be replaced by __name:=XXXXXX
  ros::NodeHandle nh;

  // wait a random time (avoid conflicts with other robots starting at the same
  // time...)
  double r = 3.0 * ((rand() % 1000) / 1000.0);
  ros::Duration wait(r); // seconds
  wait.sleep();

  double initial_x, initial_y;
  std::vector<double> list;
  nh.getParam("initial_pos", list);

  if (list.empty())
  {
    ROS_ERROR("No initial positions given: check \"initial_pos\" parameter.");
    ros::shutdown();
    exit(-1);
  }

  int value = ID_ROBOT;
  if (value == -1)
  {
    value = 0;
  }

  initial_x = list[2 * value];
  initial_y = list[2 * value + 1];

  printf("initial position: x = %f, y = %f\n", initial_x, initial_y);
  current_vertex = IdentifyVertex(vertex_web, dimension, initial_x, initial_y);
  printf("initial vertex = %d\n\n", current_vertex);

  // instantaneous idleness and last visit initialized with zeros:
  instantaneous_idleness = new double[dimension];
  last_visit = new double[dimension];

  for (size_t i = 0; i < dimension; i++)
  {
    instantaneous_idleness[i] = 0.0;
    last_visit[i] = 0.0;

    if (i == current_vertex)
    {
      last_visit[i] = 0.1; // Avoids getting back at the initial vertex
    }
    // ROS_INFO("last_visit[%d]=%f", i, last_visit[i]);
  }

  // Publicar dados de "odom" para nó de posições
  positions_pub = nh.advertise<nav_msgs::Odometry>("positions", 1); // only concerned about the most recent

  // Subscrever posições de outros robots
  positions_sub = nh.subscribe<nav_msgs::Odometry>("positions", 10, boost::bind(&PatrolAgent::positionsCB, this, _1));

  char string1[40];
  char string2[40];
  char string3[40];
  char string4[40];

  if (ID_ROBOT == -1)
  {
    strcpy(string1, "odom");    // string = "odom"
    strcpy(string2, "cmd_vel"); // string = "cmd_vel"
    strcpy(string3, "vetrex");
    strcpy(string4, "vetrex_web");
    TEAMSIZE = 1;
  }
  else
  {
    sprintf(string1, "robot_%d/odom", ID_ROBOT);
    sprintf(string2, "robot_%d/cmd_vel", ID_ROBOT);
    sprintf(string3, "robot_%d/vertex", ID_ROBOT);
    sprintf(string4, "robot_%d/vertex_web", ID_ROBOT);

    TEAMSIZE = ID_ROBOT + 1;
  }

  /* Set up listener for global coordinates of robots */
  listener = new tf::TransformListener();

  // Cmd_vel to backup:
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(string2, 1);

  // Subscrever para obter dados de "odom" do robot corrente
  odom_sub = nh.subscribe<nav_msgs::Odometry>(string1, 1, boost::bind(&PatrolAgent::odomCB, this, _1));

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  sub_to_task_planner_mission = nh.subscribe<task_planner::Task>(
      "task_planner/answer", 1, boost::bind(&PatrolAgent::receive_mission_Callback, this, _1));

  pub_to_task_planner_needtask = nh.advertise<patrolling_sim::TaskRequest>("task_planner/need_task", 1);
  pub_to_task_planner_needmission = nh.advertise<patrolling_sim::MissionRequest>("task_planner/need_mission", 1);
  pub_to_task_planner_init = nh.advertise<std_msgs::Int16MultiArray>("task_planner/init", 1);
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  pub_broadcast_msg = nh.advertise<std_msgs::Int16MultiArray>("broadcast_msg", 100);

  sub_broadcast_msg = nh.subscribe<std_msgs::Int16MultiArray>(
      "broadcast_msg", 100, boost::bind(&PatrolAgent::broadcast_msg_Callback, this, _1));

  pub_vertex_msg = nh.advertise<patrolling_sim::Vertex>(string3, 1);
  pub_vertex_web = nh.advertise<patrolling_sim::VertexWeb>(string4, 1);

  // sub_vertex_web = nh.subscribe<patrolling_sim::VertexWeb>(string4, 1,
  // boost::bind(&PatrolAgent::receive_vertex_web_Callback, this, _1));

  ros::spinOnce();

  // Publicar dados para "results"
  results_pub = nh.advertise<std_msgs::Int16MultiArray>("results", 100);
  // results_sub = nh.subscribe("results", 10, resultsCB); //Subscrever
  // "results" vindo dos robots
  results_sub = nh.subscribe<std_msgs::Int16MultiArray>("results", 100,
                                                        boost::bind(&PatrolAgent::resultsCB, this,
                                                                    _1)); // Subscrever "results" vindo dos robots

  // last time comm delay has been applied
  last_communication_delay_time = ros::Time::now().toSec();

  readParams();
}

void PatrolAgent::can_execute_decicion()
{
  std_msgs::Int16MultiArray init_msg;
  init_msg.data.clear();
  int value = ID_ROBOT;
  if (value == -1)
  {
    value = 0;
  }
  init_msg.data.push_back(value);
  init_msg.data.push_back(INIT_MSG);
  init_msg.data.push_back(current_dim_path);
  c_print("broadcast", red);
  pub_broadcast_msg.publish(init_msg);
  ros::Rate loop_rate(30);
  ros::spinOnce();
  loop_rate.sleep();
}

void PatrolAgent::calc_route_to_src()
{
  int value = ID_ROBOT;
  if (value == -1)
  {
    value = 0;
  }

  int i = 0;

  switch (value)
  {
  case (0):
  {
    i = 3;
  }
  break;
  case (1):
  {
    i = 2;
  }
  break;
  case (2):
  {
    i = 1;
  }
  break;
  case (3):
  {
    i = 0;
  }
  break;
  }

  Task t;
  t.take = true;
  Route step;
  for (int j = i; j < 5; j++)
  {
    step.id_vertex = route_to_src[j];
    step.status = false;
    t.trail.push_back(step);
  }

  mission.push_back(t);
}

int PatrolAgent::compute_cost_of_route()
{
  int anterior, proximo;

  for (int i = 1; i < mission.front().trail.size(); i++)
  {
    anterior = mission.front().trail[i - 1].id_vertex;
    proximo = mission.front().trail[i].id_vertex;

    for (int j = 0; j < vertex_web[anterior].num_neigh; j++)
    {
      if (vertex_web[anterior].id_neigh[j] == proximo)
      {
        current_dim_path += vertex_web[anterior].cost[j];
        break;
      }
    }
  }

  c_print("costo del percorso: ", current_dim_path, magenta);

  return current_dim_path;
}

void PatrolAgent::init_agent()
{

  bool f = true;
  while (!OK)
  {
    // chiedo se dopo aver calcolato la via per la src posso andare se non posso
    // aspetto
    // e dopo looprate.sleep() richiedo se posso finalmente partire
    if (f)
    {
      std_msgs::Int16MultiArray msg;
      msg.data.push_back(ID_ROBOT);
      msg.data.push_back(46);
      msg.data.push_back(CAPACITY);
      calc_route_to_src();
      compute_cost_of_route();
      pub_to_task_planner_init.publish(msg);
      f = false;
      ros::spinOnce();
      sleep(0.1);
    }
    can_execute_decicion();
  }
}

void PatrolAgent::run()
{
  // get ready
  ready();
  c_print("@ Ready!", green);

  // initially clear the costmap (to make sure the robot is not trapped):
  std_srvs::Empty srv;
  std::string mb_string;

  if (ID_ROBOT > -1)
  {
    std::ostringstream id_string;
    id_string << ID_ROBOT;
    mb_string = "robot_" + id_string.str() + "/";
  }
  mb_string += "move_base/clear_costmaps";

  if (ros::service::call(mb_string.c_str(), srv))
  {
    // if (ros::service::call("move_base/clear_costmaps", srv)){
    ROS_INFO("Costmap correctly cleared before patrolling task.");
  }
  else
  {
    ROS_WARN("Was not able to clear costmap (%s) before patrolling...", mb_string.c_str());
  }

  // Asynch spinner (non-blocking)
  ros::AsyncSpinner spinner(2); // Use n threads
  spinner.start();
  //     ros::waitForShutdown();

  /* Run Algorithm */

  for (int i = 0; i < 5; i++)
    sleep(1);

  ros::Rate loop_rate(30); // 0.033 seconds or 30Hz

  while (ros::ok())
  {
    if (goal_complete)
    {
      onGoalComplete(); // can be redefined
      resend_goal_count = 0;
    }
    else
    { // goal not complete (active)
      if (interference)
      {
        do_interference_behavior();
      }

      if (ResendGoal)
      {
        // Send the goal to the robot (Global Map)
        if (resend_goal_count < 3)
        {
          resend_goal_count++;
          ROS_INFO("Re-Sending goal (%d) - Vertex %d (%f,%f)", resend_goal_count, next_vertex,
                   vertex_web[next_vertex].x, vertex_web[next_vertex].y);
          sendGoal(next_vertex);
        }
        else
        {
          resend_goal_count = 0;
          // onGoalNotComplete();
        }
        ResendGoal = false; // para nao voltar a entrar (envia goal so uma vez)
      }

      processEvents();

      if (end_simulation)
      {
        return;
      }

    } // if (goal_complete)

    loop_rate.sleep();

  } // while ros.ok
}

int PatrolAgent::compute_next_vertex()
{
  return 666;
}

void PatrolAgent::onGoalComplete()
{
  if (next_vertex > -1)
  {
    // Update Idleness Table:
    update_idleness();
    current_vertex = next_vertex;
  }

  // devolver proximo vertex tendo em conta apenas as idlenesses;

  next_vertex = compute_next_vertex();

  c_print("   @ compute_next_vertex: ", next_vertex, green);

  // printf("Move Robot to Vertex %d (%f,%f)\n", next_vertex,
  // vertex_web[next_vertex].x, vertex_web[next_vertex].y);

  /** SEND GOAL (REACHED) AND INTENTION **/

  send_goal_reached(); // Send TARGET to monitor

  send_results(); // Algorithm specific function

  // Send the goal to the robot (Global Map)
  ROS_INFO("Sending goal - Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
  // sendGoal(vertex_web[next_vertex].x, vertex_web[next_vertex].y);
  sendGoal(next_vertex); // send to move_base

  goal_complete = false;
}

void PatrolAgent::processEvents()
{
}

void PatrolAgent::send_results()
{
}

void PatrolAgent::receive_results()
{
}

void PatrolAgent::request_Task()
{
  c_print("# Request Task!", red);

  task_request.flag = true;
  int value = ID_ROBOT;
  if (value == -1)
  {
    value = 0;
  }
  task_request.ID_ROBOT = value;
  task_request.capacity = CAPACITY;

  pub_to_task_planner_needtask.publish(task_request);

  if (first)
  {
    std_msgs::Int16MultiArray msg;
    if (value == -1)
    {
      value = 0;
    }

    msg.data.push_back(value);
    msg.data.push_back(START);
    msg.data.push_back(value + 1);
    pub_broadcast_msg.publish(msg);
    first = false;
  }

  mission.clear();

  ros::spinOnce();
  sleep(0.1);
}

} // namespace patrolagent
