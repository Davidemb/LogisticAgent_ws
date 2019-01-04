#pragma once 

namespace coo
{
    COO::COO(ros::NodeHandle &nh_)
    {
        sub_task_TP       =  nh_.subscribe("topic",1,&COO::task_TP_Callback, this);
        pub_mission_Agent =  nh_.advertise<typemessage>("topic",1);
    }

    // implementazione dei metodi 
    void COO::task_TP_Callback(const &msg)
    {
        
    }


}//namesapce coo