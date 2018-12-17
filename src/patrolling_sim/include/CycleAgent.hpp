#include "PatrolAgent.hpp"
/*
vector<Task> mission
vector<int>  route
uint id_vertex, id_task
uint loading[5] vertici superiori
uint dowload[5] vertici inferiori
*/
namespace cycleagent
{
    using namespace patrolagent;

    class CycleAgent : public PatrolAgent
    {
        public:

            // CycleAgent();
            virtual void run();
            virtual void onGoalComplete();
            virtual int compute_next_vertex();
            void compute_mission();
    };
}//namespace cycleagent

#include "impl/CycleAgent.i.hpp"