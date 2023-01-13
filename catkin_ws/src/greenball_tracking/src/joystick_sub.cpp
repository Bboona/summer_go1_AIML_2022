/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <unitree_legged_sdk/unitree_legged_sdk.h>
#include <unitree_legged_sdk/joystick.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <PahoMQTT.hpp>
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <cstdlib>
#include <ros/ros.h>


#define MAX_NODES 2

using namespace UNITREE_LEGGED_SDK;

bool running = false;

int pids[MAX_NODES];

xRockerBtnDataStruct _keyData;
// PahoMQTT::mqtt("tcp://192.168.123.161:1883", "mqtt_client"){
//     mqtt.connect();
// }

ros::Publisher pub_high;

void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr &msg)
{
    UNITREE_LEGGED_SDK::HighState state;

    for (int i(0); i < 40; i++)
    {
        state.wirelessRemote[i] = msg->wirelessRemote[i];
    }

    memcpy(&_keyData, &state.wirelessRemote[0], 40);

    if((int)_keyData.btn.components.L2 == 1 && (int)_keyData.btn.components.R2 == 1 && !running){
        // Run the greenball_tracking ROS node using the system() function
        std::cout << "Starting green_ball_tracking Node..."<< std::endl;
        running = true;
        //mqtt.setColor(0, 0, 255);

        // Create a new process for the greenball_tracking node
        pids[0] = fork();

        if (pids[0] == 0) {
            // This is the child process
            execl("/usr/bin/nohup", "/usr/bin/nohup", "rosrun", "greenball_tracking", "greenball_tracking", "1", (char*)NULL);
            exit(EXIT_SUCCESS);
        } else if (pids[0] > 0) {
            // This is the parent process
            std::cout << "Child process created with PID " << pids[0] << std::endl;
        } else {
            // fork() failed
            std::cerr << "fork() failed" << std::endl;
            exit(EXIT_FAILURE);
        }

        // Wait for the Camera
        sleep(2);
        
        // Run the second node using the same approach
        pids[1] = fork();

        if (pids[1] == 0) {
            // This is the child process
            execl("/usr/bin/nohup", "/usr/bin/nohup", "rosrun", "greenball_tracking", "movement_node", (char*)NULL);
            exit(EXIT_SUCCESS);
        } else if (pids[1] > 0) {
            // This is the parent process
            std::cout << "Child process created with PID " << pids[1] << std::endl;
        } else {
            // fork() failed
            std::cerr << "fork() failed" << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    if((int)_keyData.btn.components.L2 == 1 && (int)_keyData.btn.components.L1 == 1 && running){
        std::cout << "Stopping green_ball_tracking Node..."<< std::endl;
        //mqtt.setColor(0, 0, 0);

        for (int i = 0; i < 2; i++) {
            kill(pids[i], SIGTERM);
        }
        
        unitree_legged_msgs::HighCmd high_cmd_stop;

        for (int i = 0; i < 20; i ++){
            high_cmd_stop.head[0] = 0xFE;
            high_cmd_stop.head[1] = 0xEF;
            high_cmd_stop.levelFlag = HIGHLEVEL;
            high_cmd_stop.mode = 0;
            high_cmd_stop.gaitType = 0;
            high_cmd_stop.speedLevel = 0;
            high_cmd_stop.footRaiseHeight = 0;
            high_cmd_stop.bodyHeight = 0;
            high_cmd_stop.euler[0] = 0;
            high_cmd_stop.euler[1] = 0;
            high_cmd_stop.euler[2] = 0;
            high_cmd_stop.velocity[0] = 0.0f;
            high_cmd_stop.velocity[1] = 0.0f;
            high_cmd_stop.yawSpeed = 0.0f;
            high_cmd_stop.reserve = 0;

            pub_high.publish(high_cmd_stop);
        }

        running = false;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "node_greenball_joystick_sub");

    ros::NodeHandle nh;

    ros::Subscriber high_sub = nh.subscribe("high_state", 1, highStateCallback);

    // create a publisher to the "/high_state" topic
    pub_high = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1000);

    ros::spin();

    return 0;
}
