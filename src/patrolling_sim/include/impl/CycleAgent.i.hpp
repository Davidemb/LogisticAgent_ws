#pragma once

namespace cycleagent
{
using namespace std;

int CycleAgent::compute_next_vertex()
{
    int vertex;

    c_print("CV: ", current_vertex, green);

    if (mission[id_task].route.size() - 1 == id_vertex)
    {
        vertex = mission[id_task].route[id_vertex];
        if (mission[id_task].take)
        {
            request_Task();
        }
        else
        {
            c_print("end_simulation", red);
            end_simulation = true;
            // sleep(100);
        }
        //  ^ Importatnte!
        // mission.clear();
        c_print("id_v: ", id_vertex, " vertex: ", vertex, magenta);
        id_vertex = 0;
    }
    else
    {
        vertex = mission[id_task].route[id_vertex];
        c_print("id_v: ", id_vertex, " vertex: ", vertex, yellow);
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

    if (first)
    {
        uint elem_s_path;
        int *shortest_path = new int[dimension];
        initial_vertex = current_vertex;

        dijkstra(current_vertex, 6, shortest_path, elem_s_path, vertex_web, dimension);
        Task t;
        t.take = true;
        for (auto i = 1; i < elem_s_path; i++)
        {
            printf("path[%u] = %d\n", i, shortest_path[i]);
            t.route.push_back(shortest_path[i]);
        }
        mission.push_back(t);

        std_msgs::Int16MultiArray init_msg;
        init_msg.data.clear();
        int value = ID_ROBOT;
        if (value==-1){value=0;}
        init_msg.data.push_back(value);
        init_msg.data.push_back(INIT_MSG);

        if (elem_s_path == 1)
        {
            int cv = shortest_path[0];
            init_msg.data.push_back(cv);
            init_msg.data.push_back(cv);
        }
        else
        {
            for (int i = 0; i < elem_s_path; i++)
            {
                init_msg.data.push_back(shortest_path[i]);
                cout << shortest_path[i]<<"\n";
            }
        }
        c_print("broadcast",red);
        pub_broadcast_msg.publish(init_msg);
        ros::Rate loop_rate(30);
        ros::spinOnce();
        loop_rate.sleep();
        
        first = false;
    }

    if (OK)
    next_vertex = compute_next_vertex();
    else
    next_vertex = current_vertex;

    // next_vertex = compute_next_vertex();

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

    // auto t = mission.front();

    // cout << "route che passo alla sendmission\n";
    // for (auto i = 0; i < t.route.size(); i++)
    // {
    //     cout << t.route[i] << " ";
    // }
    // cout << "\n";

    // sendMissionGoal(t.route);

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
    // ros::waitForShutdown();


    /* Run Algorithm */

    ros::Rate loop_rate(30); // 0.033 seconds or 30Hz

    while (ros::ok())
    {
        // if (initialization)
        // {
            init_agent();

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
                    if (resend_goal_count < 2)
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
        // }// if (initialization)

        loop_rate.sleep();

    } // while ros.ok
}
} // namespace cycleagent
