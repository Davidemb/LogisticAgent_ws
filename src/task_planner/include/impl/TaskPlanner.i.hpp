#pragma once

namespace taskplanner
{
TaskPlanner::TaskPlanner(ros::NodeHandle &nh_)
{
  sub_init = nh_.subscribe("init", 1, &TaskPlanner::init_Callback, this);
  sub_task = nh_.subscribe("need_task", 1, &TaskPlanner::task_Callback, this);
  sub_task = nh_.subscribe("need_mission", 1, &TaskPlanner::mission_Callback, this);

  pub_task = nh_.advertise<task_planner::Task>("answer", 1);
  pub_results = nh_.advertise<std_msgs::Int16MultiArray>("results", 100);
  // pub_go_home = nh_advertise<>("home",1); <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // sub_mission  = nh_.subscribe("need", 1, &TaskPlanner::mission_Callback, this);
  // pub_task_to_coo = nh_.advertise<typemessage>("topic",1);
  // pub_mission = nh_.advertise<task_planner::Mission>("answer", 1);
  // t_generator();
}

void TaskPlanner::t_print(Task t)
{
  cout << "\nTask" << t.order << ":\n"
       //  << " -      take: " << t.take << "\n"
       << " -      item: " << t.item << "\n"
       //  << " -     order: " << t.order << "\n"
       << " -    demand: " << t.demand << "\n"
       //  << " -  priority: " << t.priority << "\n"
       //  << " -       src: " << t.src << "\n"
       << " -       dst: " << t.dst << "\n";
}

void TaskPlanner::pa_print(ProcessAgent pa)
{
  cout << "\nProcessAgent: \n"
       << " -  id_robot: " << pa.ID_ROBOT << "\n"
       << " -  capacity: " << pa.CAPACITY << "\n"
       << " -      flag: " << pa.flag << "\n";
}

void TaskPlanner::init(int argc, char **argv)
{
  srand(time(NULL));
  chdir(PS_path.c_str());
  mapname = string(argv[1]);
  graph_file = "/home/dave/LogisticAgent_ws/src/patrolling_sim/maps/" + mapname + "/" + mapname + ".graph";
  dimension = GetGraphDimension(graph_file.c_str());
  vertex_web = new vertex[dimension];
  GetGraphInfo(vertex_web, dimension, graph_file.c_str());
  uint nedges = GetNumberEdges(vertex_web, dimension);
  printf("Loaded graph %s with %d nodes and %d edges\n", mapname.c_str(), dimension, nedges);
  TEAM_t = atoi(argv[3]);

  t_generator();

  c_print("TEAM: ", TEAM_t, " nTask: ", nTask, magenta);
  init_agent = new bool[TEAM_t]();
  //                           ^ figo!
  pa = new ProcessAgent[TEAM_t];
}

void TaskPlanner::run()
{
}

void TaskPlanner::t_generator()
{
  uint n_item = 3;
  uint o = 0;
  uint n_demand = 3;
  // dare un id ad ogni task
  // 4 possibili partenze e destinazione
  // priorita' piu alta per i task con piu demand
  for (auto i = 0; i < n_item; i++)
  {
    for (auto d = 1; d <= n_demand; d++)
    {
      // popolo del vettore di task
      for (auto j = 0; j < 3; j++)
      {
        // auto p = d + 1;
        auto e = 0;
        tasks.push_back(mkTask(i, o, d, 1, src_vertex, dst_vertex[j], e));
        o++;
      }
    }
  }

  nTask = tasks.size();

  for (auto k = 0; k < nTask; k++)
    t_print(tasks[k]);

  std_msgs::Int16MultiArray msg;
  msg.data.clear();
  msg.data.push_back(888);  // id task_planner
  msg.data.push_back(883);  // msg type n_task
  msg.data.push_back(nTask);
  pub_results.publish(msg);
  // c_print("Pub nTask!", yellow);
  ros::spinOnce();
  sleep(0.2);
}

void TaskPlanner::compute_route_to_delivery(Task &t)
{
  route.push_back(t.src);
  status.push_back(true);
  int i = 0;
  switch (t.dst)
  {
    case 11:
      i = 3;
      break;
    case 16:
      i = 5;
      break;
    case 21:
      i = 7;
      break;
    default:
      c_print("# ERR t.dst non esiste!", red);
      break;
  }
  for (int j = 0; j < i; j++)
  {
    route.push_back(under_pass[j]);
    status.push_back(false);
  }
  route.push_back(t.dst);
  status.push_back(true);
}

void TaskPlanner::compute_route_to_picktask(Task &t)
{
  int i = 0;
  switch (t.dst)
  {
    case 11:
      i = 3;
      break;
    case 16:
      i = 5;
      break;
    case 21:
      i = 7;
      break;
    default:
      c_print("# ERR t.dst non esiste!", red);
      break;
  }
  for (int j = i - 1; j >= 0; --j)
  {
    route.push_back(upper_pass[j]);
    status.push_back(false);
  }
}

int TaskPlanner::compute_cost_of_route()
{
  int custo_final = 0;
  int anterior, proximo;

  for (int i = 1; i < route.size(); i++)
  {
    anterior = route[i - 1];
    proximo = route[i];

    for (int j = 0; j < vertex_web[anterior].num_neigh; j++)
    {
      if (vertex_web[anterior].id_neigh[j] == proximo)
      {
        custo_final += vertex_web[anterior].cost[j];
        break;
      }
    }
  }

  c_print("costo del percorso: ", custo_final, magenta);

  return custo_final;
}

void TaskPlanner::init_Callback(const std_msgs::Int16MultiArrayConstPtr &msg)
{
  /* vettore dove:
  [0] = id_robot
  [1] = type
  [2] = data
  */

  int value = msg->data[0];
  int type_msg = msg->data[1];

  if (value == -1)
  {
    value = 0;
  }

  switch (type_msg)
  {
    case (INIT_MSG):
    {
      init_agent[value] = true;
      auto c = msg->data[2];
      TEAM_c += c;
      c_print("TEAM_C: ", TEAM_c, red);
      pa[value] = mkPA(value, c);
      pa_print(pa[value]);

      uint T_t = TEAM_t;

      for (int i = 0; i < TEAM_t; i++)
      {
        if (init_agent[i] == true)
        {
          T_t--;
        }
      }
      if (T_t == 0)
      {
        // ho tutti i pa e il TEAM_c adesso devo selezionare i natblida
        conclave();
      }
    }
    break;

    default:
      break;
  }
}

void TaskPlanner::conclave()
{
  // mi preparo tutti i task in ordine
  // prima i piu vicini poi soglio ancora per capacita piu simile
  bool full = true;
  uint tmp_c = TEAM_c;
  c_print("t_c: ", tmp_c, yellow);
  c_print("T_c: ", TEAM_c, red);

  natblida.clear();

  if ((full) && (tasks.size() > 0))
  {
    int ts = tasks.size();
    for (auto i = 0; i < ts; i++)
    {
      Task t = *std::min_element(tasks.begin(), tasks.end());
      if (t.demand <= tmp_c)
      {
        tmp_c -= t.demand;
        c_print("tmpc: ", tmp_c, red);
        t.take = true;
        natblida.push_back(t);
        tasks.erase(find(tasks.begin(), tasks.end(), t));
      }
      else
      {
        full = false;
      }
    }
  }

  for (auto i = 0; i < natblida.size(); i++)
  {
    cout << "id order : " << natblida[i].order << "\n"
         << "demand   : " << natblida[i].demand << "\n"
         << "dst      : " << natblida[i].dst << "\n";
  }
}

void TaskPlanner::task_Callback(const patrolling_sim::TaskRequestConstPtr &tr)
{
  bool single_task = true;
  task_planner::Task tm;
  if ((single_task) && (tasks.size() >= 1))
  {
    Task t = *std::min_element(tasks.begin(), tasks.end());

    // Task t = *std::max_element(tasks.begin(), tasks.end());
    compute_route_to_delivery(t);
    compute_route_to_picktask(t);
    int path_distance = compute_cost_of_route();
    double normalized_distance = path_distance / t.demand;
    tm.header.stamp = ros::Time().now();
    tm.ID_ROBOT = tr->ID_ROBOT;
    tm.take = true;
    tm.go_home = false;
    tm.demand = t.demand;
    tm.item = t.item;
    tm.order = t.order;
    tm.priority = t.priority;
    tm.src = t.src;
    tm.dst = t.dst;
    tm.path_distance = path_distance;
    c_print("\nRoute:", red);
    for (auto i = 0; i < route.size(); i++)
    {
      // if (! (i % 2))
      // {
      tm.route.push_back(route[i]);
      tm.condition.push_back(status[i]);
      cout << setw(3) << route[i] << "   Status: [ " << status[i] << " ]"
           << "\n";
      // }
    }
    cout << "\n";
    c_print("% publish on topic mission! Task n: ", t.order, " ID_robot: ", tm.ID_ROBOT, yellow);
    pub_task.publish(tm);
    route.clear();
    status.clear();
    t.take = true;
    tasks.erase(std::find(tasks.begin(), tasks.end(), t));
    c_print("Size tasks: ", tasks.size(), red);
    single_task = false;
  }
  else
  {
    c_print("Task finiti!", red);
    tm.header.stamp = ros::Time().now();
    tm.ID_ROBOT = tr->ID_ROBOT;
    tm.take = false;
    tm.demand = 1;
    tm.go_home = true;
    tm.dst = initial_position[id];
    c_print("% publish on topic mission! go_home ID_robot: ", tm.ID_ROBOT, yellow);
    id++;
    pub_task.publish(tm);
    route.clear();
    status.clear();
  }
  ros::spinOnce();
  sleep(1);
}

void TaskPlanner::mission_Callback(const patrolling_sim::MissionRequestConstPtr &mr)
{ /*
   mr->capacity;
   mr->flag;
   mr->header;
   mr->ID_ROBOT;
   mr->initial_vertex;

   tasks
   natblida
   status dei vertici della route
   ProcessAgent della fase di inizializzazione - pa[nAgent]
  */
  // c_print("Request Mission! id_robot: ", mr->ID_ROBOT, green);
  // c_print(" Total capacity: ", TEAM_c, red);

  // uint id_robot = mr->ID_ROBOT;
  // bool flag = mr->flag;
  // TEAM_c += mr->capacity;

  // while (TEAM_c == 0)
  // {
  // }
}

}  // namespace taskplanner

//------------------------------------//
// void TaskPlanner::parserTask(const char *task_file)
// {
//     checkRegularFile(task_file);
//     c_print("@ Open file! path: ", task_file, green);

//     string str;

//     int tmp = 0;
//     int tmp1 = 0;
//     int tmp2 = 0;
//     int tmp3 = 0;

//     int count = 0;

//     int nEdges = 0;
//     int vertex = 0;

//     int *route = nullptr;

//     ifstream ifs(task_file);

//     if (ifs.good())
//     {
//         getline(ifs, str);
//         if (count == 0)
//         {
//             nTasks = std::atoi(str.c_str());
//             std::cout << "nTask: " << nTasks << "\n";
//         }

//         for (auto j = 0; j < nTasks; j++)
//         {
//             for (auto i = 0; i < 2; i++)
//             {
//                 // prime linee
//                 if (ifs.good())
//                 {
//                     if (i == 1)
//                     {
//                         getline(ifs, str);
//                         nEdges = std::atoi(str.c_str());
//                     }
//                     else
//                     {
//                         getline(ifs, str);
//                         std::stringstream ss(str);
//                         ss >> tmp >> tmp1 >> tmp2 >> tmp3;
//                     }
//                 }
//             }
//             // new route
//             route = new int[nEdges];
//             for (auto k = 0; k < nEdges; k++)
//             {
//                 getline(ifs, str);
//                 vertex = std::atoi(str.c_str());
//                 route[k] = vertex;
//             }
//             tasks.push_back(mkTask(tmp, tmp1, tmp2, tmp3, nEdges, route));
//         }
//         count++;
//     }
//     ifs.close();

//     // delete[] route;

//     // for (vector<Task>::iterator it = demand.begin(); it != demand.end(); ++it)
//     // {
//     //     cout << it->dimension << it->item << it->order << it->priority << it->route <<"\n";
//     // }

//     for (auto i = 0; i < tasks.size(); i++)
//         t_print(tasks[i]);
// }
//------------------------------------//

/*
double get_edge_cost_between (vertex *vertex_web, uint vertex_A, uint vertex_B){

  for (uint i=0; i<vertex_web[vertex_A].num_neigh; i++){

    if ( vertex_web[vertex_A].id_neigh[i] == vertex_B){
  return vertex_web[vertex_A].cost_m[i];
    }
  }

  return -1.0;

} */
