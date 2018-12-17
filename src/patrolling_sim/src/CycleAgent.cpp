#include "CycleAgent.hpp"

namespace cycleagent
{
void CycleAgent::compute_mission()
{
     int tmp_i = 0;

    for (auto j = 0; j < mission.size(); j++)
    {
        Task first_task = mission[j];

        if (!loading_item) 
        {
            route.push_back(first_task.src);
            loading_item = true;
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
    loading_item = false;
}
}// namesapce cycleagent