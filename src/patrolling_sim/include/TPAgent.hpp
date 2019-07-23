#include "PatrolAgent.hpp"
#include "patrolling_sim/Token.h"

namespace tpagent
{
    using namespace patrolagent;

    class TPAgent : public PatrolAgent
    {
        protected:
            ros::Publisher token_pub;
            ros::Subscriber token_sub;
        public:
            virtual void init(int argc, char **argv);
            virtual void run();
            virtual void onGoalComplete();
            virtual int compute_next_vertex();

            void token_callback(const patrolling_sim::TokenConstPtr &msg);
    };
}

#include "impl/TPAgent.i.hpp"