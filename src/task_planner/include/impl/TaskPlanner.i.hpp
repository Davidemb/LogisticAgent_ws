#pragma once

namespace taskplanner
{
TaskPlanner::TaskPlanner(ros::NodeHandle &nh_)
{
    // sub_task    = nh_.subscribe<patrolling_sim::TaskRequest>("needtask", 1, boost::bind(&TaskPlanner::task_Callback,
    // this, _1));
    sub_task = nh_.subscribe("needtask", 1, &TaskPlanner::task_Callback, this);
    pub_route = nh_.advertise<task_planner::TaskMessage>("mission", 1);
    t_generator();
    // parserTask(task_file);
    // init_agent();
}

// bool TaskPlanner::checkRegularFile(const char *task_file)
// {
//     struct stat info;
//     if (::stat(task_file, &info) != 0)
//         return false;
//     // unable to access
//     if (info.st_mode & S_IFDIR)
//         return false;
//     // is a directory
//     ifstream fin(task_file); // additional checking
//     if (!fin.is_open() || !fin.good())
//         return false;
//     try
//     {
//         // try to read
//         char c;
//         fin >> c;
//     }
//     catch (std::ios_base::failure &)
//     {
//         return false;
//     }
//     return true;
// }

void TaskPlanner::t_print(Task t)
{
    cout << "\nTask"<<t.order<<":\n"
         << " -      take: " << t.take << "\n"
         << " -      item: " << t.item << "\n"
         << " -     order: " << t.order << "\n"
         << " -    demand: " << t.demand << "\n"
         << " -  priority: " << t.priority << "\n"
         << " -       src: " << t.src << "\n"
         << " -       dst: " << t.dst << "\n"
         << " -      edge: " << t.edge<< "\n";
}

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

void TaskPlanner::task_Callback(const patrolling_sim::TaskRequestConstPtr &tr)
{
    ID_ROBOT = tr->id_robot;
    CAPACITY = tr->capacity;

    if (tr->flag)
    {
        c_print("@ Read Flag :-)", green);

        if (nTasks != tasks.size())
            c_print("### Err: nTasks != size()", red);

        for (auto i = 0; i < nTasks; i++)
        {
            task_planner::TaskMessage tm;
            if (CAPACITY >= tasks[i].demand)
            {
                if (!tasks[i].take)
                {
                    tasks[i].take = true;
                    tm.ID_ROBOT = ID_ROBOT;
                    tm.demand = tasks[i].demand;
                    tm.item = tasks[i].item;
                    tm.order = tasks[i].order;
                    tm.priority = tasks[i].priority;
                    tm.src = tasks[i].src;
                    tm.dst = tasks[i].dst;
                    tm.edge = tasks[i].edge;
                    CAPACITY -= tasks[i].demand;
                    c_print("% CPCTY: ", CAPACITY," ID_ROBOT: ", ID_ROBOT, magenta);
                    tm.header.stamp = ros::Time::now();
                    pub_route.publish(tm);
                    ROS_INFO("I published task on mission topic!");
                    sleep(3);
                }
                else
                {
                    c_print("### task taken!",red);
                }
            }
            else
            {
                c_print("# CPCTY finisched!", red);
            }
        }
        ros::spinOnce();
        sleep(1);
    }
    else
    {
        c_print("# Read Flag :-(", red);
    }
}

void TaskPlanner::init_agent()
{
    //
}

void TaskPlanner::t_generator()
{
    uint n_item = 1;
    uint o = 0;
    uint n_demand = 3;
    // dare un id ad ogni task
    // 4 possibili partenze e destinazione
    // priorita' piu alta per i task con piu demand
    src_vertex; //loading
    dst_vertex; //download
    for (auto i = 0; i < n_item; i++)
    {
        for (auto d = 1; d <= n_demand; d++)
        {
            // popolo del vettore di task
            for (auto j = 0; j < 4; j++)
            {
                auto p = d + 1;
                auto e = j+2;
                tasks.push_back(mkTask(i, o, d, p, src_vertex, dst_vertex[j],e));
                o++;
            }
        }
    }

    for (auto k = 0; k < tasks.size(); k++)
        t_print(tasks[k]);
}

} // namespace taskplanner