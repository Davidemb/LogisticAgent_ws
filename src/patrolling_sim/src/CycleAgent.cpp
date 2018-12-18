#include "CycleAgent.hpp"

namespace cycleagent
{
void CycleAgent::compute_mission()
{
    int tmp_i = 0;

    for (auto j = 0; j < mission.size(); j++)
    {
        Task first_task = mission[j];

        if (!first) 
        {
            route.push_back(first_task.src);
            first = true;
        }

        for (auto i = tmp_i; i < first_task.edge; i++)
        {
            route.push_back(downloading[i]);
            if (i == first_task.edge - 1)
                tmp_i = i;
        }
        route.push_back(first_task.dst);

        for (auto k = 0; k < route.size(); k++)
        {
            cout << route[k] << " ";
        }
        cout << "\n";
    }
    mission.clear();
    first = false;
}

int CycleAgent::go_to_src()
{
    int vertex;
    vertex = route_to_src[i];
    i++;
    return vertex;
}

void CycleAgent::compute_src(int vertex)
{

    c_print("compute src!", red);
    int i = 0;
    switch (vertex)
    {
        case 15:
            i = 5;
           
        case 12:
            i = 4;
           
        case 9:
            i = 3;
          
        case 6:
            i = 2;
    }

    c_print("I: ",i, cyan);

    for (int j = i-1; j >= 0; j--)
    {
        c_print("#######################",red);
        int element = loading[j];
        route_to_src.push_back(element);
        c_print("element: ", element, cyan);
    }
}

// void CycleAgent::share_decison()
// {
//     int value = ID_ROBOT;
//     if (value == -1) { value = 0; }

//     int msg_type = SHARE_MSG;
//     std_msgs::Int16MultiArray msg;
//     msg.data.clear();
//     msg.data.push_back(value);
//     msg.data.push_back(msg_type);
//     for (auto i = 0; i < route.size(); i++)
//     {
//         msg.data.push_back();
//     }
//     send_decision(msg);
// }
}// namesapce cycleagent