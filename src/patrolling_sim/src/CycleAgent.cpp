#include "CycleAgent.hpp"

namespace cycleagent
{
void CycleAgent::compute_mission()
{

}
    
int CycleAgent::go_to_src()
{
    return 666;
}

void CycleAgent::compute_src(int vertex)
{
   
}

void CycleAgent::compute_single_task()
{
   
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
} // namespace cycleagent