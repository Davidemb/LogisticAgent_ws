#pragma once

namespace taskplanner
{
TaskPlanner::TaskPlanner(ros::NodeHandle &nh_)
{
  sub_init = nh_.subscribe("init", 1, &TaskPlanner::init_Callback, this);
  sub_task = nh_.subscribe("need_task", 1, &TaskPlanner::task_Callback, this);
  sub_mission = nh_.subscribe("need_mission", 1, &TaskPlanner::mission_Callback, this);

  pub_task = nh_.advertise<task_planner::Task>("answer", 1);
  pub_results = nh_.advertise<std_msgs::Int16MultiArray>("results", 100);
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
  string mapname = string(argv[1]);
  string graph_file = "/home/dave/LogisticAgent_ws/src/patrolling_sim/maps/" + mapname + "/" + mapname + ".graph";
  uint dimension = GetGraphDimension(graph_file.c_str());
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
  //

  c_print ("INIT",green);
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
    // inizializzazione dei robot acquisisco Capacity
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
    // if (T_t == 0)
    // {
    //   // ho tutti i pa e il TEAM_c adesso devo selezionare i natblida
    //   skip_tasks.clear();
    //   for (auto i = 0; i < TEAM_t; i++)
    //   {
    //     for (auto i = 0; i < skip_tasks.size(); i++)
    //     {
    //       tasks.push_back(skip_tasks[i]);
    //     }
    //   //  conclave(pa[i]);
    //   }

    //   for (auto i = 0; i < skip_tasks.size(); i++)
    //   {
    //     tasks.push_back(skip_tasks[i]);
    //   }
    // }
  }
  break;

  default:
    break;
  }
}

void TaskPlanner::run()
{
}

void TaskPlanner::t_generator()
{
  uint n_item = 3;
  uint o = 0;
  uint n_demand = 3;
  for (auto i = 0; i < n_item; i++)
  {
    for (auto d = 1; d <= n_demand; d++)
    {
      for (auto j = 0; j < 3; j++)
      {
        tasks.push_back(mkTask(i, o, d, dst_vertex[j]));
        o++;
      }
    }
  }

  nTask = tasks.size();

  for (auto k = 0; k < nTask; k++)
    t_print(tasks[k]);
}

void TaskPlanner::compute_route_to_delivery(ProcessAgent &pa)
{
  // if (pa.mission.size() == 1)

  // // {

  auto el = pa;
  cout << el <<"\n";
  cout << "pa.mission.size(): " << el.mission.size() << "\n";
  Route s;
  s.id_vertex = src_vertex;
  s.status = false;
  el.route.push_back(s);
  auto dst = el.mission.front().dst;
  cout << "pa.mission.size(): " << el.mission.size() << "\n";
  int i = 0;
  switch (dst)
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
  Route step;
  for (int j = 0; j < i; j++)
  {
    step.id_vertex = under_pass[j];
    step.status = false;
    el.route.push_back(step);
  }
  step.id_vertex = dst;
  step.status = false;
  el.route.push_back(step);

   cout << el <<"\n";
}

void TaskPlanner::compute_route_to_picktask(ProcessAgent &pa)
{
  auto el = pa;
  int i = 0;
  cout << "pa.mission.size(): " << el.mission.size() << "\n";
  auto dst = el.mission.back().dst;
  switch (dst)
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
  Route step;
  for (int j = i - 1; j >= 0; --j)
  {
    step.id_vertex = upper_pass[j];
    step.status = false;
    el.route.push_back(step);
  }
  cout << el << "\n";
}

void TaskPlanner::compute_opt_delivery()
{ /* 
  //   uint dst_vertex[3] = { 11, 16, 21 };
  if (dst.size() == 2)
  {
    if (dst[0] == 11 && dst[1] == 16)
    {
      route.push_back(6);
      route.push_back(7);
      route.push_back(9);
      route.push_back(12);
      route.push_back(11);
      route.push_back(12);
      route.push_back(14);
      route.push_back(17);
      route.push_back(16);
    }
    else if (dst[0] == 11 && dst[1] == 21)
    {
      route.push_back(6);
      route.push_back(7);
      route.push_back(9);
      route.push_back(12);
      route.push_back(11);
      route.push_back(12);
      route.push_back(14);
      route.push_back(17);
      route.push_back(19);
      route.push_back(22);
      route.push_back(21);
    }
    else if (dst[0] == 16 && dst[1] == 21)
    {
      route.push_back(6);
      route.push_back(7);
      route.push_back(9);
      route.push_back(12);
      route.push_back(14);
      route.push_back(17);
      route.push_back(16);
      route.push_back(17);
      route.push_back(19);
      route.push_back(22);
      route.push_back(21);
    }
  }
  else if (dst.size() == 3)
  {
    route.push_back(6);
    route.push_back(7);
    route.push_back(9);
    route.push_back(12);
    route.push_back(11);
    route.push_back(12);
    route.push_back(14);
    route.push_back(17);
    route.push_back(16);
    route.push_back(17);
    route.push_back(19);
    route.push_back(22);
    route.push_back(21);
  }
  else
  {
    c_print("ERR!", red);
  } */
}

int TaskPlanner::compute_cost_of_route(ProcessAgent &pa)
{
  auto el = pa;
  int custo_final = 0;
  int anterior, proximo;

  cout << "pa.route.size(): " << el.route.size() << "\n";

  for (int i = 1; i < el.route.size(); i++)
  {
    anterior = el.route[i - 1].id_vertex;
    proximo = el.route[i].id_vertex;

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

void TaskPlanner::conclave(ProcessAgent pa)
{ /* 
  // mi preparo tutti i task in ordine
  // prima i piu vicini poi soglio ancora per capacita piu simile
  bool full = true;
  uint tmp_c = pa.CAPACITY;
  c_print("t_c: ", tmp_c, yellow);

  route.clear();
  skip_tasks.clear();

  if ((full) && (tasks.size() > 0))
  {
    int ts = tasks.size();
    for (auto i = 0; i < ts; i++)
    {
      Task t = *std::min_element(tasks.begin(), tasks.end(), pop_min_element);
      c_print("t.demand: ", t.demand, magenta);
      if (t.demand <= tmp_c)
      {
        tmp_c -= t.demand;
        c_print("tmpc: ", tmp_c, red);
        t.take = true;
        pa.total_demand += t.demand;
        pa.total_item[t.item] += t.demand;
        pa.mission.push_back(t);
        tasks.erase(find(tasks.begin(), tasks.end(), t));
      }
      else
      {
        if (pa.mission.size() <= 0)
        {
          skip_tasks.push_back(t);
          tasks.erase(find(tasks.begin(), tasks.end(), t));
          c_print("ERR! no allocato task", red);
        }
        full = false;
      }
    }
  }
  // calcolo dei percoorsi
  dst.clear();
  for (auto i = 0; i < pa.mission.size(); i++)
  {
    auto el = pa.mission[i];
    dst.push_back(el.dst);
  }

  sort(dst.begin(), dst.end());
  dst.erase(unique(dst.begin(), dst.end()), dst.end());

  if (dst.size() == 1)
  {
    compute_route_to_delivery(pa.mission.front());
    compute_route_to_picktask(pa.mission.front());
  }
  else
  {
    compute_opt_delivery();
    compute_route_to_picktask(pa.mission.back());
  }

  // return route e status
  c_print("\nRoute: ", red);
  for (auto i = 0; i < route.size(); i++)
  {
    pa.route.push_back(route[i]);
    pa.status.push_back(false);
    cout << route[i] << " ";
  }
  cout << "\n";
  cout << pa << "\n";
  for (auto i = 0; i < skip_tasks.size(); i++)
  {
    tasks.push_back(skip_tasks[i]);
  } */
}

void TaskPlanner::task_Callback(const patrolling_sim::TaskRequestConstPtr &tr)
{
  bool single_task = true;
  uint id_robot = tr->ID_ROBOT;
  auto el = pa[id_robot];

  task_planner::Task tm;

  c_print("DIO CANE",red);
  cout << el << "\n";

  if ((single_task) && (tasks.size() >= 1))
  {
    Task t = *std::min_element(tasks.begin(), tasks.end());
    cout << el << "\n";
    cout << "task dst: "<< t.dst << "\n";
    // Task t = *std::max_element(tasks.begin(), tasks.end());
    el.mission.push_back(t);
    cout << el << "\n";
    cout << "size: " << el.mission.size() << "\n";
    // cout << el <<"\n";
    compute_route_to_delivery(el);
    // cout << el << "\n";
    compute_route_to_picktask(el);
    for (auto i = 0; i < el.route.size(); i++)
    {
      cout << setw(3) << el.route[i].id_vertex << "   Status: [ " << el.route[i].status << " ]"
           << "\n";
    }
    cout << "\n";
    int path_distance = compute_cost_of_route(el);
    tm.header.stamp = ros::Time().now();
    tm.ID_ROBOT = tr->ID_ROBOT;
    tm.take = true;
    tm.go_home = false;
    tm.demand = t.demand;
    tm.item = t.item;
    tm.order = t.order;
    tm.dst = t.dst;
    tm.path_distance = path_distance;
    c_print("\nRoute:", red);
    for (auto i = 0; i < el.route.size(); i++)
    {
      tm.route.push_back(el.route[i].id_vertex);
      tm.condition.push_back(el.route[i].status);
      cout << setw(3) << el.route[i].id_vertex << "   Status: [ " << el.route[i].status << " ]"
           << "\n";
    }
    cout << "\n";
    c_print("% publish on topic mission! Task n: ", t.order, " ID_robot: ", tm.ID_ROBOT, yellow);
    pub_task.publish(tm);
    el.mission.clear();
    el.route.clear();
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
    el.mission.clear();
    el.route.clear();
  }
  ros::spinOnce();
  sleep(1);
}

void TaskPlanner::mission_Callback(const patrolling_sim::MissionRequestConstPtr &mr)
{
  task_planner::Task tm;
  c_print("Request Mission! id_robot: ", mr->ID_ROBOT, green);
  uint id_robot = mr->ID_ROBOT;

  // route.clear();
  pa[id_robot].mission.clear();
  pa[id_robot].route.clear();

  conclave(pa[id_robot]);

  bool flag = mr->flag;
  tm.ID_ROBOT = id_robot;
  tm.take = true;
  tm.go_home = false;
  // tm.item = pa[id_robot].total_item[0];
  tm.demand = pa[id_robot].total_demand;

  c_print("for ", yellow);

  for (auto i = 0; i < pa[id_robot].route.size(); i++)
  {
    cout << pa[id_robot] << " ";
    tm.route.push_back(pa[id_robot].route[i].id_vertex);
    tm.condition.push_back(pa[id_robot].route[i].status);
  }
  cout << "\n";

  c_print("pub mission", yellow);
  // preparo i pa per i prossimi task

  pub_task.publish(tm);
  ros::spinOnce();
  sleep(1);
}

} // namespace taskplanner
