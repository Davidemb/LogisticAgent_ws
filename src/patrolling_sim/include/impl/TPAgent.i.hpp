#pragma once
#include "patrolling_sim/Token.h"

namespace tpagent
{

void TPAgent::init(int argc, char **argv)
{
    PatrolAgent::init(argc, argv);
    ros::NodeHandle nh;

    token_pub = nh.advertise<patrolling_sim::Token>("token_msg", 1);

    token_sub = nh.subscribe<patrolling_sim::Token>("token_msg", 20,
        boost::bind(&TPAgent::token_callback, this, _1));
}

void TPAgent::run()
{
    // get ready
    ready();

    c_print("@ Ready!", green);

    // initially clear the costmap (to make sure the robot is not trapped):
    std_srvs::Empty srv;
    std::string mb_string;

    if (ID_ROBOT > -1)
    {
        std::ostringstream id_string;
        id_string << ID_ROBOT;
        mb_string = "robot_" + id_string.str() + "/";
    }
    mb_string += "move_base/clear_costmaps";

    if (ros::service::call(mb_string.c_str(), srv))
    {
        // if (ros::service::call("move_base/clear_costmaps", srv)){
        ROS_INFO("Costmap correctly cleared before patrolling task.");
    }
    else
    {
        ROS_WARN("Was not able to clear costmap (%s) before patrolling...", mb_string.c_str());
    }

    // Asynch spinner (non-blocking)
    ros::AsyncSpinner spinner(2); // Use n threads
    spinner.start();
    // ros::waitForShutdown();

    /* Run Algorithm */

    //  init_agent3();

    // c_print("RequestTask", green);
    // request_Task();
    sleep(5);

    ros::Rate loop_rate(30); // 0.033 seconds or 30Hz

    /*
     *  ogni bot manda la teamsize ipotizzata in broadcast
     *  e attende 1 secondo, in questo modo ogni bot scopre
     *  la reale teamsize
     * (migliorabile)
     */
    c_print("Pubblico teamsize", green);
    patrolling_sim::Token t;
    t.ID_ROBOT = ID_ROBOT;
    t.TEAMSIZE = ID_ROBOT+1;
    t.INIT_DONE = false;
    token_pub.publish(t);
    sleep(5);

    if(ID_ROBOT == 0) {
        t.ID_ROBOT = 0;
        t.TEAMSIZE = TEAMSIZE;
        t.INIT_DONE = true;
        token_pub.publish(t);
    }

    while (ros::ok())
    {
        loop_rate.sleep();

    } // while ros.ok
}

void TPAgent::onGoalComplete()
{
}

int TPAgent::compute_next_vertex()
{
}

void TPAgent::token_callback(const patrolling_sim::TokenConstPtr &msg)
{
    //calcolo dimensione team
    if(!msg->INIT_DONE)
    {
        if(msg->TEAMSIZE > TEAMSIZE)
        {
            TEAMSIZE = msg->TEAMSIZE;
        }
    }
    else if(ID_ROBOT == (msg->ID_ROBOT+1) % TEAMSIZE)
    {
        std::ostringstream oss;
        oss << "Token ricevuto! ID messaggio: " << msg->ID_ROBOT
                << "\tID robot: " << ID_ROBOT << "\tTEAMSIZE: " << TEAMSIZE;
        std::string s = oss.str();
        c_print(s.c_str(), green);
        patrolling_sim::Token t;
        t.ID_ROBOT = ID_ROBOT;
        t.INIT_DONE = true;
        t.TEAMSIZE = TEAMSIZE;
        ros::Duration d(1.0);
        d.sleep();
        token_pub.publish(t);
        c_print("Token pubblicato!", green);
    }
}

} //namespace tpagent