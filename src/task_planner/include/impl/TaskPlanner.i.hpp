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

void TaskPlanner::t_print(Task &t)
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
        // task_set.insert(mkTask(i,o,d,dst_vertex[j]));
        o++;
      }
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
  /* //   uint dst_vertex[3] = { 11, 16, 21 };
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
  }  */
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
  //     Task t = *std::min_element(tasks.begin(), tasks.end(), pop_min_element);
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

void TaskPlanner::ps_print(int s[], int size)
{
  for (int i = 1; i <= size; i++)
    std::cout << s[i] - 1 << " ";
  std::cout << std::endl;

  // return;
}

/* s[] = gli indici, k e m = 0, n = size */

// void TaskPlanner::powerSet(int s[], int k, int m, int n)
// {
//   if (m <= n)
//   {
//     s[k + 1] = m;
//     ps_print(s, k + 1);
//     powerSet(s, k + 1, m + 1, n);
//     powerSet(s, k, m + 1, n);
//   }
// }
/*
#include <iostream>
#include <set>

template <class S>
auto powerset(const S& s)
{
    std::set<S> ret;
    ret.emplace();
    for (auto&& e: s) {
        std::set<S> rs;
        for (auto x: ret) {
            x.insert(e);
            rs.insert(x);
        }
        ret.insert(begin(rs), end(rs));
    }
    return ret;
}

int main()
{
    std::set<int> s = {222, 3131, 25, 7};
    auto pset = powerset(s);

    for (auto&& subset: pset) {
        std::cout << "{ ";
        char const* prefix = "";
        for (auto&& e: subset) {
            std::cout << prefix << e;
            prefix = ", ";
        }
        std::cout << " }\n";
    }
} */

//
/* 
#include <iostream>
#include <set>
#include <vector>
#include <iterator>
#include <algorithm>
typedef std::set<int> set_type;
typedef std::set<set_type> powerset_type;
 
powerset_type powerset(set_type const& set)
{
  typedef set_type::const_iterator set_iter;
  typedef std::vector<set_iter> vec;
  typedef vec::iterator vec_iter;
 
  struct local
  {
    static int dereference(set_iter v) { return *v; }
  };
 
  powerset_type result;
 
  vec elements;
  do
  {
    set_type tmp;
    std::transform(elements.begin(), elements.end(),
                   std::inserter(tmp, tmp.end()),
                   local::dereference);
    result.insert(tmp);
    if (!elements.empty() && ++elements.back() == set.end())
    {
      elements.pop_back();
    }
    else
    {
      set_iter iter;
      if (elements.empty())
      {
        iter = set.begin();
      }
      else
      {
        iter = elements.back();
        ++iter;
      }
      for (; iter != set.end(); ++iter)
      {
        elements.push_back(iter);
      }
    }
  } while (!elements.empty());
 
  return result;
}
 
int main()
{
  int values[4] = { 2, 3, 5, 7 };
  set_type test_set(values, values+4);
 
  powerset_type test_powerset = powerset(test_set);
 
  for (powerset_type::iterator iter = test_powerset.begin();
       iter != test_powerset.end();
       ++iter)
  {
    std::cout << "{ ";
    char const* prefix = "";
    for (set_type::iterator iter2 = iter->begin();
         iter2 != iter->end();
         ++iter2)
    {
      std::cout << prefix << *iter2;
      prefix = ", ";
    }
    std::cout << " }\n";
  }
} */

set<set<Task>> TaskPlanner::powerSet(const set<Task> &t)
{
  typedef set<Task>::const_iterator set_iter;
  typedef vector<set_iter> vec;
  typedef vec::iterator vec_iter;

  struct local
  {
    static Task dereference(set_iter v) { return *v; }
  };

  set<set<Task>> result;
  vec elements;
  do
  {
    set<Task> tmp;
    transform(elements.begin(), elements.end(), inserter(tmp, tmp.end()), local::dereference);
    result.insert(tmp);
    if (!elements.empty() && ++elements.back() == t.end())
    {
      elements.pop_back();
    }
    else
    {
      set_iter iter;
      if (elements.empty())
      {
        iter = t.begin();
      }
      else
      {
        iter = elements.back();
        ++iter;
      }
      for (; iter != t.end(); ++iter)
      {
        elements.push_back(iter);
      }
    }
  } while (!elements.empty());

  return result;
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

  bool full = true;
  task_planner::Task tm;
  uint id_robot = mr->ID_ROBOT;
  uint tmp_c = TEAM_c;
  auto el = &pa[id_robot];
  //        ^ importante!

  // estrapolo i natblida rispetto la capacita' totale
  // poi calcolo tuttte le possibili combinazioni di task

  // natblida.clear();
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
        c_print("tmp_c: ", tmp_c, red);
        t.take = true;
        c_print("insert natblida, id_order:", t.order, green);
        task_set.insert(t);
        tasks.erase(find(tasks.begin(), tasks.end(), t));
      }
      else
      {
        if (task_set.size() <= 0)
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

  auto pset = powerSet(task_set);

  cout << "pset.size(): " << pset.size() << "\n";
  cout << "task_set.size(): " << task_set.size() << "\n";

  uint id = 0;

  bool f = false;

  for (set<set<Task>>::iterator iter = pset.begin(); iter != pset.end(); ++iter)
  {
    std::cout << "{ ";
    char const *prefix = "";
    uint temp_c = 0;
    ProcessTask pt;
    for (set<Task>::iterator iter2 = iter->begin(); iter2 != iter->end(); ++iter2)
    {
      std::cout << prefix << *iter2;
      prefix = ", ";
      temp_c += iter2->demand;
      pt.id = id;
      pt.mission.push_back(*iter2);
    }
    id++;
    std::cout << " } demand: " << temp_c << "\n";
    pt.tot_demand = temp_c;
    if (f)
    {
      v_pt.push_back(pt);
    }
    f = true;
  }

  for (auto i = 0; i < v_pt.size(); i++)
  {
    cout << "id: " << v_pt[i].id << "\n ";
    for (auto j = 0; j < v_pt[i].mission.size(); j++)
    {
      cout << v_pt[i].mission[j].order << " ";
    }
    cout << "\n";
  }

  auto max_el = *std::max_element(pa, pa + TEAM_t);
  cout << "elemento massimo " << max_el.CAPACITY << "\n";

  int c = 0;
  for (vector<ProcessTask>::iterator it = v_pt.begin(); it != v_pt.end(); ++it)
  {
    // if (it->tot_demand > max_el.CAPACITY)
    // {
      // cout << "dc: "<<c<<"\n";
      v_pt.erase(find(v_pt.begin(), v_pt.end(), *it));
      c++;
    // }
  }

  c_print("porca troia dio cane ", magenta);

  for (auto i = 1; i < v_pt.size(); i++)
  {
    cout << "id: " << v_pt[i].id << "\n ";
    for (auto j = 0; j < v_pt[i].mission.size(); j++)
    {
      cout << v_pt[i].mission[j].order << " ";
    }
    cout << "\ndemand: " << v_pt[i].tot_demand << "\n";
  }
}

} // namespace taskplanner
