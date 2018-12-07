#pragma once

namespace taskplanner
{
TaskPlanner::TaskPlanner(ros::NodeHandle &nh_)
{
    // sub_task    = nh_.subscribe<patrolling_sim::TaskRequest>("needtask", 1, boost::bind(&TaskPlanner::task_Callback,
    // this, _1));
    sub_task = nh_.subscribe("needtask", 1, &TaskPlanner::task_Callback, this);
    pub_route = nh_.advertise<task_planner::TaskMessage>("mission", 1);
    parserTask(task_file);
    // init_agent();
}

bool TaskPlanner::checkRegularFile(const char *task_file)
{
    struct stat info;
    if (::stat(task_file, &info) != 0)
        return false;
    // unable to access
    if (info.st_mode & S_IFDIR)
        return false;
    // is a directory
    ifstream fin(task_file); // additional checking
    if (!fin.is_open() || !fin.good())
        return false;
    try
    {
        // try to read
        char c;
        fin >> c;
    }
    catch (std::ios_base::failure &)
    {
        return false;
    }
    return true;
}

void TaskPlanner::t_print(Task t)
{
    cout << "\nTask :\n"
         << " -      take: " << t.take << "\n"
         << " -      item: " << t.item << "\n"
         << " -     order: " << t.order << "\n"
         << " -    demand: " << t.demand << "\n"
         << " -  priority: " << t.priority << "\n"
         << " - dimension: " << t.dimension << "\n"
         << " -     route:";

    for (auto i = 0; i < t.dimension; i++)
    {
        cout << " " << t.route[i];
    }
    cout << "\n";
}

void TaskPlanner::parserTask(const char *task_file)
{
    checkRegularFile(task_file);
    c_print("@ Open file! path: ", task_file, green);

    string str;

    int tmp = 0;
    int tmp1 = 0;
    int tmp2 = 0;
    int tmp3 = 0;

    int count = 0;

    int nEdges = 0;
    int vertex = 0;

    int *route = nullptr;

    ifstream ifs(task_file);

    if (ifs.good())
    {
        getline(ifs, str);
        if (count == 0)
        {
            nTasks = std::atoi(str.c_str());
            std::cout << "nTask: " << nTasks << "\n";
        }

        for (auto j = 0; j < nTasks; j++)
        {
            for (auto i = 0; i < 2; i++)
            {
                // prime linee
                if (ifs.good())
                {
                    if (i == 1)
                    {
                        getline(ifs, str);
                        nEdges = std::atoi(str.c_str());
                    }
                    else
                    {
                        getline(ifs, str);
                        std::stringstream ss(str);
                        ss >> tmp >> tmp1 >> tmp2 >> tmp3;
                    }
                }
            }
            // new route
            route = new int[nEdges];
            for (auto k = 0; k < nEdges; k++)
            {
                getline(ifs, str);
                vertex = std::atoi(str.c_str());
                route[k] = vertex;
            }
            tasks.push_back(mkTask(tmp, tmp1, tmp2, tmp3, nEdges, route));
        }
        count++;
    }
    ifs.close();

    // delete[] route;

    // for (vector<Task>::iterator it = demand.begin(); it != demand.end(); ++it)
    // {
    //     cout << it->dimension << it->item << it->order << it->priority << it->route <<"\n";
    // }

    for (auto i = 0; i < tasks.size(); i++)
        t_print(tasks[i]);
}

void TaskPlanner::task_Callback(const patrolling_sim::TaskRequestConstPtr &tr)
{
    ID_ROBOT = tr->id_robot;
    // BOL_FLAG = tr.flag;
    CAPACITY = tr->capacity;
    // fare controllo sulla capacita'

    int temp_CPCTY = 0;

    if (tr->flag)
    {
        c_print("@ Read Flag :-)", green);

        int id_vertex;

        tm.route.clear();

        if (nTasks != tasks.size())
            c_print("### Err: nTasks != size()", red);

        for (auto i = 0; i < nTasks; i++)
        {
            if (!tasks[i].take)
            {
                if (CAPACITY <= tasks[i].demand)
                {
                    for (auto j = 0; j < tasks[i].dimension; j++)
                    {
                        tasks[i].take = true;
                        id_vertex = tasks[i].route[j];
                        tm.route.push_back(id_vertex);
                        tm.demand = tasks[i].demand;
                        tm.dimension = tasks[i].dimension;
                        tm.item = tasks[i].item;
                        tm.order = tasks[i].order;
                        tm.priority = tasks[i].priority;
                        // tasks.pop_back();
                    }
                    CAPACITY -= tasks[i].demand;
                }
            }
            pub_route.publish(tm);
            ROS_INFO("I published task on mission topic!");
            sleep(3);
        }
        ros::spinOnce();
        sleep(2);
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

} // namespace taskplanner