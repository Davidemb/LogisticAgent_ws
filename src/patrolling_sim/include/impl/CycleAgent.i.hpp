#pragma once

namespace cycleagent
{
using namespace std;

int CycleAgent::compute_next_vertex()
{
    int vertex;

    if (route.size()-1 == id_vertex)
    {
        vertex = route[id_vertex];
        request_Task();

        c_print ("id_v: ",id_vertex, " vertex: ", vertex, yellow);

        route.clear();
        id_vertex = 0;
        go_src = true;
        mission_complete = false;
    }
    else
    {
        vertex = route[id_vertex];
        c_print ("id_v: ",id_vertex, " vertex: ", vertex, yellow);
        id_vertex++;
    }

    return vertex;
}

void CycleAgent::onGoalComplete()
{
    if (next_vertex > -1)
    {
        // Update Idleness Table:
        update_idleness();
        current_vertex = next_vertex;
    }

    // devolver proximo vertex tendo em conta apenas as idlenesses;
    if (!go_src)
    {
        c_print("go_src: ",go_src, red);
        next_vertex = compute_next_vertex();
        last_vertex = next_vertex;
    }
    else
    {
        c_print("go_src!", cyan);
        compute_src(last_vertex);
        next_vertex = go_to_src();
    }
    c_print("   @ compute_next_vertex: ", next_vertex, green);

    // printf("Move Robot to Vertex %d (%f,%f)\n", next_vertex,
    // vertex_web[next_vertex].x, vertex_web[next_vertex].y);

    /** SEND GOAL (REACHED) AND INTENTION **/

    send_goal_reached(); // Send TARGET to monitor

    send_results(); // Algorithm specific function

    // Send the goal to the robot (Global Map)
    ROS_INFO("Sending goal - Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
    // sendGoal(vertex_web[next_vertex].x, vertex_web[next_vertex].y);
    sendGoal(next_vertex); // send to move_base

    goal_complete = false;
}

void CycleAgent::run()
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
    //     ros::waitForShutdown();

    /* Run Algorithm */

    for (int i = 0; i < 5; i++)
        sleep(1);

    ros::Rate loop_rate(30); // 0.033 seconds or 30Hz

    while (ros::ok())
    {
        if (!mission_complete)
        {
            compute_mission();
            mission_complete = true;
        }
        if (goal_complete)
        {
            onGoalComplete(); // can be redefined
            resend_goal_count = 0;
        }
        else
        { // goal not complete (active)
            if (interference)
            {
                do_interference_behavior();
            }

            if (ResendGoal)
            {
                // Send the goal to the robot (Global Map)
                if (resend_goal_count < 3)
                {
                    resend_goal_count++;
                    ROS_INFO("Re-Sending goal (%d) - Vertex %d (%f,%f)", resend_goal_count, next_vertex,
                             vertex_web[next_vertex].x, vertex_web[next_vertex].y);
                    sendGoal(next_vertex);
                }
                else
                {
                    resend_goal_count = 0;
                    // onGoalNotComplete();
                }
                ResendGoal = false; // para nao voltar a entrar (envia goal so uma vez)
            }

            processEvents();

            if (end_simulation)
            {
                return;
            }

        } // if (goal_complete)

        loop_rate.sleep();

    } // while ros.ok
}
} // namespace cycleagent
