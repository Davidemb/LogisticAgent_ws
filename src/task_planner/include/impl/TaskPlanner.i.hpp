#pragma once

namespace taskplanner
{
TaskPlanner::TaskPlanner(ros::NodeHandle &nh_, uint TEAMSIZE)
{
  TEAMSIZE        = TEAM_t;
  sub_task        = nh_.subscribe("need", 1, &TaskPlanner::task_Callback, this);
  pub_task        = nh_.advertise<task_planner::Task>("answer", 1);
  // sub_mission  = nh_.subscribe("need", 1, &TaskPlanner::mission_Callback, this);
  // pub_task_to_coo = nh_.advertise<typemessage>("topic",1);
  pub_mission     = nh_.advertise<task_planner::Mission>("answer",1); 
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
       << " -       dst: " << t.dst << "\n"
       << " -      edge: " << t.edge << "\n";
}

// void TaskPlanner::task_Callback(const patrolling_sim::TaskRequestConstPtr &tr)
// {
//     ID_ROBOT = tr->id_robot;
//     CAPACITY = tr->capacity;

//     if (tr->flag)
//     {
//         c_print("@ Request activate!", green);

//         for (auto i = 0; i < nTasks; i++)
//         {
//             task_planner::TaskMessage tm;
//             if (CAPACITY >= tasks[i].demand)
//             {
//                 if (!tasks[i].take)
//                 {
//                     tasks[i].take = true;
//                     tm.ID_ROBOT = ID_ROBOT;
//                     tm.demand = tasks[i].demand;
//                     tm.item = tasks[i].item;
//                     tm.order = tasks[i].order;
//                     tm.priority = tasks[i].priority;
//                     tm.src = tasks[i].src;
//                     tm.dst = tasks[i].dst;
//                     tm.edge = tasks[i].edge;
//                     CAPACITY -= tasks[i].demand;
//                     c_print("% id_task: ", tm.order, " ID_ROBOT: ", tm.ID_ROBOT, yellow);
//                     tm.header.stamp = ros::Time::now();
//                     pub_route.publish(tm);
//                     ROS_INFO("I published task on mission topic!");
//                     sleep(3);
//                 }
//                 else
//                 {
//                     c_print("### task taken!", red);
//                 }
//             }
//             else
//             {
//                 c_print("# CPCTY finisched!", red);
//                 break;
//             }
//         }
//         ros::spinOnce();
//         sleep(1);
//     }
//     else
//     {
//         c_print("# Read Flag :-(", red);
//     }
// }

Task TaskPlanner::compare(Task t1, Task t2)
{
  return t1.dst < t2.dst ? t1 : t2 ;
}

void TaskPlanner::task_Callback(const patrolling_sim::TaskRequestConstPtr &tr)
{
  bool single_task = true;
  arrived_message = new bool[TEAM_t];
  all_capacity += tr->capacity; 
  if (tr->flag)
  {
    for (vector<Task>::iterator it = tasks.begin(); it != tasks.end(); it++)
    {
      if ((!it->take) && (single_task))
      {
        it->take = true;
        task_planner::Task tm;
        if ((tr->capacity <= it->demand))
        {
          // for (vector<Task>::iterator te = tasks.begin(); te != tasks.end(); te++)
          // {
          //   Task t = compare(*it,*te);
          //   cout << "vedem: "<< t.dst <<" id: "<<t.order << "\n";
          // }
          Task t = *std::min_element(tasks.begin(), tasks.end());
          cout << "allor: "<< t.dst<<" id: "<<t.order<<"\n";
          // id del task e' anche id nel vettore!!!!!!!!!!!!!!!!!!!!!!!!11!111!1!!!!!!
        } 
        // {
        //   Task t = *std::min_element(tasks.begin(),tasks.end(),compare());
        //   cout << "dst del minimo elemto: " <<t.dst << "\n";
        // }
        arrived_message[tr->ID_ROBOT] = true;
        tm.header.stamp = ros::Time().now();
        tm.ID_ROBOT = tr->ID_ROBOT;
        tm.demand = it->demand;
        tm.item = it->item;
        tm.order = it->order;
        tm.priority = it->priority;
        tm.src = it->src;
        tm.dst = it->dst;
        tm.edge = it->edge;
        c_print("% publish on topic mission! Task n: ", it->order," ID_robot: ",tm.ID_ROBOT, yellow);
        pub_task.publish(tm);
        single_task = false;
        sleep(3);
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

void TaskPlanner::t_generator()
{
  uint n_item = 1;
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
      for (auto j = 0; j < 4; j++)
      {
        // auto p = d + 1;
        auto e = j + 2;
        tasks.push_back(mkTask(i, o, 1, 1, src_vertex, dst_vertex[j], e));
        o++;
      }
    }
  }

  // nTasks = tasks.size();

  for (auto k = 0; k < tasks.size(); k++)
    t_print(tasks[k]);
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