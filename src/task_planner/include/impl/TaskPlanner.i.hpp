#pragma once

namespace taskplanner
{

TaskPlanner::TaskPlanner(ros::NodeHandle &nh_, uint TEAMSIZE)
{
  TEAMSIZE = TEAM_t;
  sub_task = nh_.subscribe("need_task", 1, &TaskPlanner::task_Callback, this);
  // sub_task = nh_.subscribe("need_mission", 1, &TaskPlanner::mission_Callback, this);
  pub_task = nh_.advertise<task_planner::Task>("answer", 1);
  // sub_mission  = nh_.subscribe("need", 1, &TaskPlanner::mission_Callback, this);
  // pub_task_to_coo = nh_.advertise<typemessage>("topic",1);
  // pub_mission = nh_.advertise<task_planner::Mission>("answer", 1);
  t_generator();
}

void TaskPlanner::t_print(Task t)
{
  cout << "\nTask" << t.order << ":\n"
       << " -      take: " << t.take << "\n"
       << " -      item: " << t.item << "\n"
       << " -     order: " << t.order << "\n"
       << " -    demand: " << t.demand << "\n"
       << " -  priority: " << t.priority << "\n"
       << " -       src: " << t.src << "\n"
       << " -       dst: " << t.dst << "\n";
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
}

void TaskPlanner::run()
{
}

Task TaskPlanner::compare(Task t1, Task t2)
{
  return t1.dst < t2.dst ? t1 : t2;
}

// void TaskPlanner::mission_Callback(const patrolling_sim::MissionRequestConstPtr &tr)
// {
//   c_print("Funge?", magenta);
//   // arrived_message = new bool[TEAM_t];
//   // all_capacity += tr->capacity;
//   if (tr->flag)
//   {
//     task_planner::Mission mm;
//     for (vector<Task>::iterator it = tasks.begin(); it != tasks.end(); it++)
//     {
//       if (!it->take)
//       {
//         it->take = true;
//         task_planner::Task tm;
//         if ((tr->capacity <= it->demand))
//         {
//           Task t = *std::min_element(tasks.begin(), tasks.end());
//           cout << "allor: " << t.dst << " id: " << t.order << "\n";
//           // id del task e' anche id nel vettore!!!!!!!!!!!!!!!!!!!!!!!!11!111!1!!!!!!
//           // arrived_message[tr->ID_ROBOT] = true;
//           tm.header.stamp = ros::Time().now();
//           tm.ID_ROBOT = tr->ID_ROBOT;
//           tm.demand = t.demand;
//           tm.item = t.item;
//           tm.order = t.order;
//           tm.priority = t.priority;
//           tm.src = t.src;
//           tm.dst = t.dst;
//           // tm.edge = t.edge;
//           c_print("% publish on topic mission! Task n: ", t.order, " ID_robot: ", tm.ID_ROBOT, yellow);
//           c_print("% take: ", t.take, yellow);
//           tasks.erase(std::find(tasks.begin(), tasks.end(), t));
//           c_print("DC", magenta);
//           mm.Mission.push_back(tm);  // <-
//           c_print("popolazione dell'array", red);
//           sleep(1);
//         }
//       }
//     }
//     // c_print("all_C: ",all_capacity, green);
//     // pub_mission.publish(mm);
//     for (auto i = 0; i < mm.Mission.size(); i++)
//     {
//       cout << mm.Mission[i].order << "\n";
//     }
//     ros::spinOnce();
//     sleep(1);
//   }
//   else
//   {
//     c_print("# task taken!", red);
//   }
// }

void TaskPlanner::t_generator()
{
  uint n_item = 3;
  uint o = 0;
  uint n_demand = 1;
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
        tasks.push_back(mkTask(i, o, 1, 1, src_vertex, dst_vertex[j], e));
        o++;
      }
    }
  }

  for (auto k = 0; k < tasks.size(); k++)
    t_print(tasks[k]);
}

void TaskPlanner::compute_route_to_delivery(Task &t)
{
  route.push_back(t.src);
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
  }
  route.push_back(t.dst);
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
  }
}

int TaskPlanner::compute_cost_of_route(uint element)
{
  int custo_final = 0;
  int anterior, proximo;

  for (int i = 1; i < element; i++)
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

  c_print("costo del percorso? ", custo_final, magenta);

  return custo_final;
}

void TaskPlanner::task_Callback(const patrolling_sim::TaskRequestConstPtr &tr)
{
  bool single_task = true;
  // arrived_message = new bool[TEAM_t];
  // all_capacity += tr->capacity;
  if (tr->flag)
  {
    task_planner::Task tm;
    for (vector<Task>::iterator it = tasks.begin(); it != tasks.end(); it++)
    {
      if ((!it->take) && (single_task))
      {
        if ((tr->capacity <= it->demand))
        {
          int c = 0;

          Task t = *std::min_element(tasks.begin(), tasks.end());
          compute_route_to_delivery(t);
          compute_route_to_picktask(t);
          int path_distance = compute_cost_of_route(route.front());
          double normalized_distance = path_distance / t.demand;
          tm.header.stamp = ros::Time().now();
          tm.ID_ROBOT = tr->ID_ROBOT;
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
            if (! (i % 2))
            {
            tm.route.push_back(route[i]);
            cout << route[i] << "\n";
            }
          }
          cout << "\n";
          c_print("% publish on topic mission! Task n: ", t.order, " ID_robot: ", tm.ID_ROBOT, yellow);
          pub_task.publish(tm);
          route.clear();
          t.take = true;
          tasks.erase(std::find(tasks.begin(), tasks.end(), t));
          single_task = false;
          // ros::spinOnce();
          sleep(3);
        }
      }
    }
    // c_print("all_C: ",all_capacity, green);
    ros::spinOnce();
    sleep(1);
  }
  else
  {
    c_print("# task taken!", red);
  }
}

} // namespace taskplanner

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
