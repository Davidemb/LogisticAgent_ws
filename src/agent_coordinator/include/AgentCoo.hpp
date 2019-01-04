#pragma once
#include <...>

namespace coo
{
    class COO
    {
        public:
            COO(ros::NodeHandle &nh_);
            ~COO();

            void task_TP_Callback(const typemessagePtr &msg);

        private:
            ros::Subscriber sub_task_TP;
            ros::Publisher  pub_mission_Agent;
    }
}// namespace coo

#include "impl/AgentCoo.i.hpp"