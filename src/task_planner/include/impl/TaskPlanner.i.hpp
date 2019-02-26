#pragma once

namespace taskplanner
{
static bool OK = false;

TaskPlanner::TaskPlanner(ros::NodeHandle &nh_)
{
  sub_init = nh_.subscribe("init", 1, &TaskPlanner::init_Callback, this);
  sub_task = nh_.subscribe("need_task", 1, &TaskPlanner::task_Callback, this);
  sub_mission = nh_.subscribe("need_mission", 1, &TaskPlanner::mission_Callback, this);

  pub_task = nh_.advertise<task_planner::Task>("answer", 1);
  pub_results = nh_.advertise<std_msgs::Int16MultiArray>("results", 100);
}

void TaskPlanner::t_print(Task &t)
{
  cout << "\nTask" << t.order << ":\n"
       << " -      item: " << t.item << "\n"
       << " -    demand: " << t.demand << "\n"
       << " -       dst: " << t.dst << "\n";
}

void TaskPlanner::pa_print(ProcessAgent &pa)
{
  cout << "\nProcessAgent: \n"
       << " -  id_robot: " << pa.ID_ROBOT << "\n"
       << " -  capacity: " << pa.CAPACITY << "\n"
       << " -      flag: " << pa.flag << "\n"
       << "mission.size: " << pa.mission.size() << "\n"
       << " -     route: ";
  for (auto i = 0; i < pa.route.size(); i++)
  {
    cout << pa.route[i].id_vertex << " ";
  }
  cout << "\n";
}

void TaskPlanner::ct_print(CandidateTask &ct)
{
  cout << "\nCandidateTask: \n"
       << " -        id: " << ct.id << "\n"
       << " -    subset: " << ct.subset << "\n"
       << " - candidate: ";
  cout << "{";
  for (auto i = 0; i < ct.subset; i++)
  {
    auto el = ct.vv_t[i];
    cout << "(";
    for (auto j = 0; j < el.size() - 1; j++)
    {
      cout << el[j] << " ";
    }
    cout << el.back() << ")";
  }
  cout << "}"
       << "\n";
  cout << " -         V: " << ct.V << "\n";
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

  c_print("INIT", green);
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
    if (T_t == 0)
    {
      OK = true;
      compute_best_subtask();
      auto ele = pq_ct.top();

      ct_print(ele);

      cout << "good? " << ele.good << "\n";
      cout << ele << "\n";
      for (auto i = 0; i < ele.subset; i++)
      {
        auto s_el = ele.vv_t[i].size();
        auto el = ele.vv_t[i];
        for (auto j = 0; j < s_el; j++)
        {
          auto e = el[j].route;
          auto r = el[j];
          v_pt.push_back(r);
          for (auto k = 0; k < e.size(); k++)
          {
            cout << e[k] << " ";
          }
          cout << "\n";
        }
      }

      for (auto h = 0; h < v_pt.size(); h++)
      {
        auto el = v_pt[h].route;
        cout << "ROute: \n";
        for (auto g = 0; g < el.size(); g++)
        {
          cout << el[g] << " ";
        }
        cout << "\n";
      }

      c_print("fine", green);
    }
  }
  break;

  default:
    break;
  }
}

void TaskPlanner::run()
{
  // check partition:
  // vector<ProcessTask> v_pt;
}

void TaskPlanner::t_generator()
{
  uint n_item = 3;
  uint o = 0;
  uint n_demand = 3;
  uint j = 0;
  for (auto d = 1; d <= n_demand; d++)
  {
    j = 0;
    for (auto i = 0; i < n_item; i++)
    {
      tasks.push_back(mkTask(i, o, d, dst_vertex[j]));
      j++;
      // task_set.insert(mkTask(i,o,d,dst_vertex[j]));
      o++;
    }
  }
  nTask = tasks.size();
  for (auto k = 0; k < nTask; k++)
    t_print(tasks[k]);
}

void TaskPlanner::compute_route_to_delivery(ProcessAgent *pa)
{
  // if (pa.mission.size() == 1)

  // // {
  Route step;
  auto el = pa;
  cout << el << "\n";
  cout << "pa.mission.size(): " << el->mission.size() << "\n";

  step.id_vertex = src_vertex;
  step.status = true;
  el->route.push_back(step);
  auto dst = el->mission.front().dst;
  cout << "pa.mission.size(): " << el->mission.size() << "\n";
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
  // Route step;
  for (int j = 0; j < i; j++)
  {
    step.id_vertex = under_pass[j];
    step.status = false;
    // tmp_route.push_back(step);
    pa->route.push_back(step);
  }
  step.id_vertex = dst;
  step.status = true;
  // tmp_route.push_back(step);
  pa->route.push_back(step);
  cout << el << "\n";
}

void TaskPlanner::compute_route_to_picktask(ProcessAgent *pa)
{
  auto el = pa;
  Route step;
  int i = 0;
  cout << "pa.mission.size(): " << el->mission.size() << "\n";
  auto dst = el->mission.back().dst;
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
  for (int j = i - 1; j >= 0; --j)
  {
    step.id_vertex = upper_pass[j];
    step.status = false;
    // tmp_route.push_back(step);
    pa->route.push_back(step);
  }
  cout << el << "\n";
}

void TaskPlanner::compute_opt_delivery()
{
  // TODO!
}

int TaskPlanner::compute_cost_of_route(ProcessAgent *pa)
{
  auto el = pa;
  int custo_final = 0;
  // int anterior, proximo;

  cout << "pa.route.size(): " << pa->route.size() << "\n";

  for (int i = 1; i < pa->route.size(); i++)
  {
    int anterior = pa->route[i - 1].id_vertex;
    int proximo = pa->route[i].id_vertex;

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

int TaskPlanner::ccor(vector<uint> route)
{
  int custo_final = 0;
  for (int i = 1; i < route.size(); i++)
  {
    int anterior = route[i - 1];
    int proximo = route[i];

    for (int j = 0; j < vertex_web[anterior].num_neigh; j++)
    {
      if (vertex_web[anterior].id_neigh[j] == proximo)
      {
        custo_final += vertex_web[anterior].cost[j];
        break;
      }
    }
  }
  return custo_final;
}

void TaskPlanner::conclave(ProcessAgent &pa)
{
  // // mi preparo tutti i task in ordine
  // // prima i piu vicini poi soglio ancora per capacita piu simile
  // bool full = true;
  // uint tmp_c = pa.CAPACITY;
  // c_print("t_c: ", tmp_c, yellow);

  // skip_tasks.clear();

  // if ((full) && (tasks.size() > 0))
  // {
  //   int ts = tasks.size();
  //   for (auto i = 0; i < ts; i++)
  //   {
  //     Task t = *std::min_element(tasks.begin(), tasks.end(),
  //     pop_min_element);
  //     c_print("t.demand: ", t.demand, magenta);
  //     if (t.demand <= tmp_c)
  //     {
  //       tmp_c -= t.demand;
  //       c_print("tmp_c: ", tmp_c, red);
  //       t.take = true;
  //       pa.total_demand += t.demand;
  //       pa.total_item[t.item] += t.demand;
  //       auto t_d = pa.total_demand;
  //       cout << "t_D: " << t_d << "\n";
  //       c_print("insert task on mission", green);
  //       pa.mission.push_back(t);
  //       tasks.erase(find(tasks.begin(), tasks.end(), t));
  //     }
  //     else
  //     {
  //       if (pa.mission.size() <= 0)
  //       {
  //         skip_tasks.push_back(t);
  //         tasks.erase(find(tasks.begin(), tasks.end(), t));
  //         c_print("ERR! no allocato task", red);
  //       }
  //       full = false;
  //     }
  //   }
  // }
  // // calcolo dei percoorsi
  // dst.clear();
  // for (auto i = 0; i < pa.mission.size(); i++)
  // {
  //   auto el = pa.mission[i];
  //   dst.push_back(el.dst);
  // }

  // sort(dst.begin(), dst.end());
  // dst.erase(unique(dst.begin(), dst.end()), dst.end());

  // if (dst.size() == 1)
  // {
  //   compute_route_to_delivery(pa);
  //   compute_route_to_picktask(pa);
  // }
  // else
  // {
  //   compute_opt_delivery();
  //   compute_route_to_picktask(pa);
  // }

  // // return route e status
  // // c_print("\nRoute: ", red);
  // // for (auto i = 0; i < route.size(); i++)
  // // {
  // //   // pa.route.push_back(route[i]);
  // //   pa.status.push_back(false);
  // //   // cout << route[i] << " ";
  // // }
  // // cout << "\n";
  // cout << pa << "\n";
  // for (auto i = 0; i < skip_tasks.size(); i++)
  // {
  //   tasks.push_back(skip_tasks[i]);
  // }
}

uint TaskPlanner::compute_cycle_dst(vector<Task> mission)
{
  /*
   1 =11
   2 = 16
   3 = 21
   4 = 11-16
   5 = 11-21
   6 = 16-21
   7 = 11-16-21 */
  uint res = 0;
  vector<uint> tmp_d;
  tmp_d.clear();
  for (auto i = 0; i < mission.size(); i++)
  {
    auto d = mission[i].dst;
    tmp_d.push_back(d);
  }
  sort(tmp_d.begin(), tmp_d.end());
  tmp_d.erase(unique(tmp_d.begin(), tmp_d.end()), tmp_d.end());

  if (tmp_d.size() == 1)
  {
    if (tmp_d[0] == 11)
      res = 1;
    else if (tmp_d[0] == 16)
      res = 2;
    else
      res = 3;
  }
  else if (tmp_d.size() == 2)
  {
    if ((tmp_d[0] == 11) && (tmp_d[1] == 16))
    {
      res = 4;
    }
    else if ((tmp_d[0] == 11) && (tmp_d[1] == 21))
    {
      res = 5;
    }
    else
    {
      res = 6;
    }
  }
  else
  {
    res = 7;
  }

  return res;
}

void TaskPlanner::compute_route(uint id_path, ProcessTask *pt)
{
  auto ele = pt;
  /* 1 =11
     2 = 16
     3 = 21
     4 = 11-16
     5 = 11-21
     6 = 16-21
     7 = 11-16-21 */
  switch (id_path)
  {
  case 1:
  {
    for (auto i = 0; i < 8; i++)
    {
      // v_pt[j].route.push_back(p_11[i]);
      ele->route.push_back(p_11[i]);
    }
    // v_pt[j].path_distance = ccor(v_pt[j].route);
    ele->path_distance = ccor(ele->route);
    ele->V = (double)ele->path_distance / (double)ele->tot_demand;
    // c_print("V: ", ele->V, red);
  }
  break;
  case 2:
  {
    for (auto i = 0; i < 12; i++)
    {
      ele->route.push_back(p_16[i]);
    }
    ele->path_distance = ccor(ele->route);
    // ele->V = ele->path_distance / ele->tot_demand;
    ele->V = (double)ele->path_distance / (double)ele->tot_demand;
    // c_print("V: ", ele->V, red);
  }
  break;
  case 3:
  {
    for (auto i = 0; i < 16; i++)
    {
      ele->route.push_back(p_21[i]);
    }
    ele->path_distance = ccor(ele->route);
    // ele->V = ele->path_distance / ele->tot_demand;
    ele->V = (double)ele->path_distance / (double)ele->tot_demand;
    // c_print("V: ", ele->V, red);
  }
  break;
  case 4:
  {
    for (auto i = 0; i < 14; i++)
    {
      ele->route.push_back(p_11_16[i]);
    }
    ele->path_distance = ccor(ele->route);
    ele->V = (double)ele->path_distance / (double)ele->tot_demand;
    // ele->V = ele->path_distance / ele->tot_demand;
    // c_print("V: ", ele->V, red);
  }
  break;

  case 5:
  {
    for (auto i = 0; i < 18; i++)
    {
      ele->route.push_back(p_11_21[i]);
    }
    ele->path_distance = ccor(ele->route);
    ele->V = (double)ele->path_distance / (double)ele->tot_demand;
    // ele->V = ele->path_distance / ele->tot_demand;
    // c_print("V: ", ele->V, red);
  }
  break;
  case 6:
  {
    for (auto i = 0; i < 18; i++)
    {
      ele->route.push_back(p_16_21[i]);
    }
    ele->path_distance = ccor(ele->route);
    ele->V = (double)ele->path_distance / (double)ele->tot_demand;
    // ele->V = ele->path_distance / ele->tot_demand;
    // c_print("V: ", ele->V, red);
  }
  break;
  case 7:
  {
    for (auto i = 0; i < 20; i++)
    {
      ele->route.push_back(p_11_16_21[i]);
    }
    ele->path_distance = ccor(ele->route);
    ele->V = (double)ele->path_distance / (double)ele->tot_demand;
    // ele->V = ele->path_distance / ele->tot_demand;
    // c_print("V: ", ele->V, red);
  }
  break;
  default:
  {
    c_print("ERR", red);
  }
  break;
  }
}

vector<Task> TaskPlanner::processedTask()
{
  bool full = true;

  uint tmp_c = TEAM_c;

  skip_tasks.clear();

  if ((full) && (tasks.size() > 0))
  {
    int ts = tasks.size();
    for (auto i = 0; i < ts; i++)
    {
      Task t = *std::min_element(tasks.begin(), tasks.end(), pop_min_element);
      // c_print("t.demand: ", t.demand, magenta);
      if (t.demand <= tmp_c)
      {
        tmp_c -= t.demand;
        // c_print("tmp_c: ", tmp_c, red);
        t.take = true;
        c_print("insert natblida, id_order:", t.order, green);
        c_print("with ddemand: ", t.demand, green);
        // task_set.insert(t);
        natblida.push_back(t);
        tasks.erase(find(tasks.begin(), tasks.end(), t));
      }
      else
      {
        if (natblida.size() <= 0)
        {
          skip_tasks.push_back(t);
          tasks.erase(find(tasks.begin(), tasks.end(), t));
          c_print("ERR! no allocato task", red);
        }
        full = false;
      }
    }
  }
  // ciclo di aggiornamento
  for (auto i = 0; i < skip_tasks.size(); i++)
  {
    tasks.push_back(skip_tasks[i]);
  }

  return natblida;
}

void TaskPlanner::prepare_missions()
{
  auto max_el = *std::max_element(pa, pa + TEAM_t);
  cout << "elemento massimo " << max_el.CAPACITY << "\n";
  auto t_size = tasks.size();
  try
  {
    partition::iterator it(t_size);
    int id = 0;
    while (true)
    {
      auto all_part = *it[tasks];
      auto subset = it.subsets();
      auto as = all_part.size();
      auto ct = mkCT(id, subset);
      id++;
      int pt_id = 0;
      double tmp_V = 0;
      for (auto k = 0; k < subset; k++)
      {
        int tmp_d = 0;
        auto part = all_part[k];
        // un processTask e' un subset di Task
        ProcessTask pt;
        pt.id = pt_id;
        pt_id++;
        for (auto j = 0; j < part.size(); j++)
        {
          tmp_d += part[j].demand;
          pt.mission.push_back(part[j]);
        }
        pt.tot_demand = tmp_d;
        if (tmp_d > max_el.CAPACITY)
          ct.good++;
        uint id_path = compute_cycle_dst(pt.mission);
        compute_route(id_path, &pt);
        tmp_V += pt.V;
        ct.vv_t[k].push_back(pt);
      }
      ++it;
      ct.V = tmp_V;
      v_ct.push_back(ct);
    }
  }
  catch (std::overflow_error &)
  {
  }
  c_print("fine preparazione!", red);
}

void TaskPlanner::skip_task(CandidateTask &ct)
{
  // cout << "\nCandidateTask: \n"
  //      << " -        id: " << ct.id << "\n"
  //      << " -    subset: " << ct.subset << "\n"
  //      << " - candidate: ";
  // cout << "{";
  // for (auto i = 0; i < ct.subset; i++) {
  //   auto el = ct.vv_t[i];
  //   int tmp_d = 0;
  //   cout << "(";
  //   for (auto j = 0; j < el.size(); j++) {
  //     cout << el[j] << " ";
  //     tmp_d += el[j].demand;
  //   }
  //   cout << ") "
  //        << "T_d: ";
  //   if (tmp_d > 3) {
  //     cout << " SKIP< " << tmp_d << " >";

  //   } else {
  //     cout << tmp_d;
  //   }
  // }
  // cout << "}"
  //      << "\n";
}

void TaskPlanner::h_value(CandidateTask &ct)
{
  // auto subset = ct.subset;
  // for (auto i = 0; i < subset; i++)
  // {
  //   auto el = ct.vv_t[i];
  //   int tmp = 0;
  //   auto path = compute_cycle_dst(el);
  //     for (auto j = 0; j < el.size(); j++)
  //     {
  //       tmp += el[j].demand;
  //     }
  //   double V = path / tmp;
  // }
}

void TaskPlanner::compute_best_subtask()
{
  prepare_missions();

  // for (auto i = 0; i < v_ct.size(); i++) {
  //   auto ct = v_ct[i];
  //   c_print("estraggo l'elemento id: ", ct.id, green);
  //   for (auto j = 0; j < ct.subset; j++) {
  //     auto ele = ct.vv_t[j];
  //     int tmp_d = 0;
  //     for (auto k = 0; k < ele.size(); k++) {
  //       cout << ele[k].order << "\n";
  //       tmp_d += ele[k].demand;
  //     }
  //     if (tmp_d > max_el.CAPACITY) {
  //       c_print("tmp_d: ", tmp_d, red);
  //       c_print("SKIP", red);
  //       c_print("ID: ", ct.id, magenta);
  //       v_ct.erase(find(v_ct.begin(), v_ct.end(), ct));
  //       // tmp_d = 0;
  //       // break;
  //       cout << "\n\n\n";
  //     }
  //   }
  // }
  // for (auto i = 0; i < v_ct.size(); i++) {
  //   auto el = v_ct[i];
  //   ct_print(el);
  // }

  // for (vector<CandidateTask>::iterator it = v_ct.begin(); it != v_ct.end();)
  // {
  //   auto id = it->id;
  //   cout << "Id: " << id;
  //   auto subset = it->subset;
  //   for (auto i = 0; i < subset; i++) {
  //     auto e = it->vv_t[i];
  //     int tmp_d = 0;
  //     for (auto j = 0; j < e.size(); j++) {
  //       tmp_d += e[j].demand;
  //     }

  //     if (tmp_d > max_el.CAPACITY) {
  //       cout << " tmp_d: " << tmp_d << "\n";
  //       v_ct.erase(find(v_ct.begin(), v_ct.end(), *it));
  //       break;
  //     }
  //   }
  //   v_good.push_back(*it);
  //   ++it;
  // }

  for (vector<CandidateTask>::iterator it = v_ct.begin(); it != v_ct.end();)
  {
    if (it->good == 0)
    {
      // cout << it->good << "\n";
      // v_good.push_back(*it);
      pq_ct.push(*it);
    }
    else
    {
      // c_print("nada :", it->good, red);
    }
    ++it;
  }

  v_ct.clear();

  // c_print("GOOD", green);
  // while (!pq_ct.empty())
  // {
  //   cout << pq_ct.top() << " ";
  //   pq_ct.pop();
  // }
  // cout << "\n";
  // c_print("VCT", magenta);
  // for (auto i = 0; i < v_ct.size(); i++) {
  //   auto el = v_ct[i];
  //   ct_print(el);
  // }
}

void TaskPlanner::task_Callback(const patrolling_sim::TaskRequestConstPtr &tr)
{
  bool single_task = true;
  uint id_robot = tr->ID_ROBOT;
  auto el = &pa[id_robot];
  //        ^ importante!

  task_planner::Task tm;

  if ((single_task) && (tasks.size() >= 1))
  {
    Task t = *std::min_element(tasks.begin(), tasks.end(), pop_min_element);

    el->mission.push_back(t);

    compute_route_to_delivery(el);
    compute_route_to_picktask(el);
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
    for (auto i = 0; i < el->route.size(); i++)
    {
      tm.route.push_back(el->route[i].id_vertex);
      tm.condition.push_back(el->route[i].status);
      cout << setw(3) << el->route[i].id_vertex << "   Status: [ " << el->route[i].status << " ]"
           << "\n";
    }
    cout << "\n";
    c_print("% publish on topic mission! Task n: ", t.order, " ID_robot: ", tm.ID_ROBOT, yellow);
    pub_task.publish(tm);
    el->mission.clear();
    el->route.clear();
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
    tm.demand = 0;
    tm.item = 3;
    tm.go_home = true;
    tm.dst = initial_position[id];
    c_print("% publish on topic mission! go_home ID_robot: ", tm.ID_ROBOT, yellow);
    id++;
    pub_task.publish(tm);
    el->mission.clear();
    el->route.clear();
  }
  ros::spinOnce();
  sleep(1);
}

void TaskPlanner::mission_Callback(const patrolling_sim::MissionRequestConstPtr &mr)
{
  c_print("Request Mission! id_robot: ", mr->ID_ROBOT, green);

  // prepare_missions();
  // compute_best_subtask();

  bool single_task = true;
  uint id_robot = mr->ID_ROBOT;
  task_planner::Task tm;

  if ((single_task) && (v_pt.size() >= 1))
  {
    ProcessTask pt = v_pt.front();
    tm.header.stamp = ros::Time().now();
    tm.ID_ROBOT = id_robot;
    tm.take = true;
    tm.go_home = false;
    tm.demand = pt.tot_demand;
    tm.item = 0; // per ora!
    tm.order = pt.id;
    tm.dst = 0; // per ora
    tm.path_distance = pt.path_distance;
    for (auto i = 0; i < pt.route.size(); i++)
    {
      tm.route.push_back(pt.route[i]);
      tm.condition.push_back(false);
    }
    pub_task.publish(tm);
    v_pt.erase(find(v_pt.begin(), v_pt.end(), pt));
    single_task = false;
  }
  else
  {
    tm.header.stamp = ros::Time().now();
    tm.ID_ROBOT = mr->ID_ROBOT;
    tm.take = false;
    tm.demand = 0;
    tm.item = 3;
    tm.go_home = true;
    tm.dst = initial_position[id];
    c_print("% publish on topic mission! go_home ID_robot: ", tm.ID_ROBOT, yellow);
    id++;
    pub_task.publish(tm);
  }

  ros::spinOnce();
  sleep(1);

  // for (auto k = 0; k < v_good.size(); k++)
  // {
  //   auto el = &v_good[k];  // Funghe?
  //   auto subset = el->subset;
  //   for (auto q = 0; q < subset; q++)
  //   {
  //     auto e = el->vv_t[q];
  //     for (auto t = 0; t < e.size(); t++)
  //     {
  //       auto ele = &e[t];
  //       uint id_path = compute_cycle_dst(ele->mission);
  //       switch (id_path)
  //       {
  //         case 1:
  //         {
  //           for (auto i = 0; i < 8; i++)
  //           {
  //             // v_pt[j].route.push_back(p_11[i]);
  //             ele->route.push_back(p_11[i]);
  //           }
  //           // v_pt[j].path_distance = ccor(v_pt[j].route);
  //           ele->path_distance = ccor(ele->route);
  //           ele->V = ele->path_distance / ele->tot_demand;
  //         }
  //         break;
  //         case 2:
  //         {
  //           for (auto i = 0; i < 12; i++)
  //           {
  //             ele->route.push_back(p_16[i]);
  //           }
  //           ele->path_distance = ccor(ele->route);
  //           ele->V = ele->path_distance / ele->tot_demand;
  //         }
  //         break;
  //         case 3:
  //         {
  //           for (auto i = 0; i < 16; i++)
  //           {
  //             ele->route.push_back(p_21[i]);
  //           }
  //           ele->path_distance = ccor(ele->route);
  //           ele->V = ele->path_distance / ele->tot_demand;
  //         }
  //         break;
  //         case 4:
  //         {
  //           for (auto i = 0; i < 14; i++)
  //           {
  //             ele->route.push_back(p_11_16[i]);
  //           }
  //           ele->path_distance = ccor(ele->route);
  //           ele->V = ele->path_distance / ele->tot_demand;
  //         }
  //         break;

  //         case 5:
  //         {
  //           for (auto i = 0; i < 18; i++)
  //           {
  //             ele->route.push_back(p_11_21[i]);
  //           }
  //           ele->path_distance = ccor(ele->route);
  //           ele->V = ele->path_distance / ele->tot_demand;
  //         }
  //         break;
  //         case 6:
  //         {
  //           for (auto i = 0; i < 18; i++)
  //           {
  //             ele->route.push_back(p_16_21[i]);
  //           }
  //           ele->path_distance = ccor(ele->route);
  //           ele->V = ele->path_distance / ele->tot_demand;
  //         }
  //         break;
  //         case 7:
  //         {
  //           for (auto i = 0; i < 20; i++)
  //           {
  //             ele->route.push_back(p_11_16_21[i]);
  //           }
  //           ele->path_distance = ccor(ele->route);
  //           ele->V = ele->path_distance / ele->tot_demand;
  //         }
  //         break;
  //         default:
  //         {
  //           c_print("ERR", red);
  //         }
  //         break;
  //       }
  //     }
  //   }
  // }

  c_print("fine", red);

  // task_planner::Task tm;
  // uint id_robot = mr->ID_ROBOT;
  // uint tmp_c = TEAM_c;
  // auto el = &pa[id_robot];
  // //        ^ importante!

  // auto nat = processedTask();

  // int size = nat.size();

  // // optional second argument is the partition size (1--size)

  // try
  // {
  //   partition::iterator it(size);

  //   while (true)
  //   {
  //     auto all_part = *it[nat];

  //     // cout << all_part << "\n";

  //     auto as = all_part.size();

  //     int c = 0;

  //     for (auto i = 0; i < as; i++)
  //     {
  //       auto part = all_part[i];
  //       ProcessTask pt;
  //       c++;
  //       for (auto j = 0; j < part.size(); j++)
  //       {
  //         pt.id = c;
  //         pt.mission.push_back(part[j]);
  //         cout << part[j] << " ";
  //       }
  //       cout << " \n";

  //       v_pt.push_back(pt);
  //     }

  //     ++it;
  //   }
  // }
  // catch (std::overflow_error &)
  // {
  // }

  // auto pset = powerSet(task_set);

  // cout << "pset.size(): " << pset.size() << "\n";
  // cout << "task_set.size(): " << task_set.size() << "\n";

  // uint id = 0;

  // bool f = false;

  // for (set<set<Task>>::iterator iter = pset.begin(); iter != pset.end();
  // ++iter)
  // {
  //   std::cout << "{ ";
  //   char const *prefix = "";
  //   uint temp_c = 0;
  //   ProcessTask pt;
  //   for (set<Task>::iterator iter2 = iter->begin(); iter2 != iter->end();
  //   ++iter2)
  //   {
  //     std::cout << prefix << *iter2;
  //     prefix = ", ";
  //     temp_c += iter2->demand;
  //     pt.id = id;
  //     pt.mission.push_back(*iter2);
  //   }
  //   id++;
  //   std::cout << " } demand: " << temp_c << "\n";
  //   pt.tot_demand = temp_c;
  //   if (f)
  //   {
  //     v_pt.push_back(pt);
  //   }
  //   f = true;
  // }

  // c_print("v_pt.size(); ", v_pt.size(), red);

  // for (auto i = 0; i < v_pt.size(); i++)
  // {
  //   cout << "id: " << v_pt[i].id << "\n ";
  //   for (auto j = 0; j < v_pt[i].mission.size(); j++)
  //   {
  //     cout << v_pt[i].mission[j].order << " ";
  //   }
  //   cout << "\n";
  // }

  // auto max_el = *std::max_element(pa, pa + TEAM_t);
  // cout << "elemento massimo " << max_el.CAPACITY << "\n";

  // for (vector<ProcessTask>::iterator it = v_pt.begin(); it != v_pt.end();)
  // {
  //   cout << "id: " << it->id << " demand:" << it->tot_demand << "\n";
  //   if (it->tot_demand > max_el.CAPACITY)
  //   {
  //     c_print("SKIP", red);
  //     cout << "id: " << it->id << " demand:" << it->tot_demand << "\n";
  //     v_pt.erase(find(v_pt.begin(), v_pt.end(), *it));
  //   }
  //   else
  //   {
  //     ++it;
  //   }
  // }

  // for (auto i = 0; i < v_pt.size(); i++)
  // {
  //   cout << "id: " << v_pt[i].id << "\n ";
  //   for (auto j = 0; j < v_pt[i].mission.size(); j++)
  //   {
  //     cout << v_pt[i].mission[j].order << " ";
  //   }
  //   cout << "\ndemand: " << v_pt[i].tot_demand << "\n";
  // }

  // // calcolo il percorso diviso per il totdemand il percorso parte dalla
  // sorgente
  // // destinazioni e di nuovo sorgente
  // for (auto j = 0; j < v_pt.size(); j++)
  // {
  //   uint id_path = compute_cycle_dst(v_pt[j].mission);
  //   switch (id_path)
  //   {
  //     case 1:
  //     {
  //       for (auto i = 0; i < 8; i++)
  //       {
  //         v_pt[j].route.push_back(p_11[i]);
  //       }
  //       v_pt[j].path_distance = ccor(v_pt[j].route);
  //     }
  //     break;
  //     case 2:
  //     {
  //       for (auto i = 0; i < 12; i++)
  //       {
  //         v_pt[j].route.push_back(p_16[i]);
  //       }
  //       v_pt[j].path_distance = ccor(v_pt[j].route);
  //     }
  //     break;
  //     case 3:
  //     {
  //       for (auto i = 0; i < 16; i++)
  //       {
  //         v_pt[j].route.push_back(p_21[i]);
  //       }
  //       v_pt[j].path_distance = ccor(v_pt[j].route);
  //     }
  //     break;
  //     case 4:
  //     {
  //       for (auto i = 0; i < 14; i++)
  //       {
  //         v_pt[j].route.push_back(p_11_16[i]);
  //       }
  //       v_pt[j].path_distance = ccor(v_pt[j].route);
  //     }
  //     break;

  //     case 5:
  //     {
  //       for (auto i = 0; i < 18; i++)
  //       {
  //         v_pt[j].route.push_back(p_11_21[i]);
  //       }
  //       v_pt[j].path_distance = ccor(v_pt[j].route);
  //     }
  //     break;
  //     case 6:
  //     {
  //       for (auto i = 0; i < 18; i++)
  //       {
  //         v_pt[j].route.push_back(p_16_21[i]);
  //       }
  //       v_pt[j].path_distance = ccor(v_pt[j].route);
  //     }
  //     break;
  //     case 7:
  //     {
  //       for (auto i = 0; i < 20; i++)
  //       {
  //         v_pt[j].route.push_back(p_11_16_21[i]);
  //       }
  //       v_pt[j].path_distance = ccor(v_pt[j].route);
  //     }
  //     break;
  //     default:
  //     {
  //       c_print("ERR", red);
  //     }
  //     break;
  //   }
  // }

  // for (auto i = 0; i < v_pt.size(); i++)
  // {
  //   cout << "id: " << v_pt[i].id << "\n ";
  //   for (auto j = 0; j < v_pt[i].mission.size(); j++)
  //   {
  //     cout << v_pt[i].mission[j].order << " ";
  //   }
  //   cout << "\ndemand: " << v_pt[i].tot_demand << "\n";
  //   cout << "route: ";
  //   for (auto k = 0; k < v_pt[i].route.size(); k++)
  //   {
  //     cout << v_pt[i].route[k] << " ";
  //   }
  //   cout << "\n";
  //   cout << "PD: " << v_pt[i].path_distance << "\n";

  //   p_q.push(v_pt[i]);
  // }

  // auto t = p_q.top();

  // c_print("PQ: ", red);
  // while (!p_q.empty())
  // {
  //   cout << p_q.top() << " ";
  //   p_q.pop();
  // }
  // cout << "\n";

  // tm.ID_ROBOT = id_robot;
  // tm.demand = t.tot_demand;
  // tm.path_distance = t.path_distance;

  // for (auto i = 0; i < t.route.size(); i++)
  // {
  //   cout << t.route[i] << " ";
  //   tm.route.push_back(t.route[i]);
  // }
  // cout << "\n";

  // pub_task.publish(tm);
  // ros::spinOnce();
  // sleep(1);
}

} // namespace taskplanner
