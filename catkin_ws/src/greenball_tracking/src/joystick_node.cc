/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <unitree_legged_sdk/unitree_legged_sdk.h>
#include <unitree_legged_sdk/joystick.h>
#include <PahoMQTT.hpp>
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <cstdlib>
#include <ros/ros.h>
#include <ros/node_handle.h>

using namespace UNITREE_LEGGED_SDK;

bool running = false;
int pid = 0;  // Initialize the child process ID to 0

class Joystick
{
public:
    Joystick(uint8_t level): 
        safe(LeggedType::Go1), 
        udp(level, 8090, "192.168.123.161", 8082),
        mqtt("tcp://192.168.123.161:1883", "mqtt_client"){
        udp.InitCmdData(cmd);
        mqtt.connect();
    }
    void UDPSend();
    void UDPRecv();
    void RobotControl();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    xRockerBtnDataStruct _keyData;
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
    PahoMQTT mqtt;
};

void Joystick::UDPRecv()
{ 
    udp.Recv();
}

void Joystick::UDPSend()
{  
    udp.Send();
}

void Joystick::RobotControl() 
{
    motiontime++;
    udp.GetRecv(state);

    memcpy(&_keyData, &state.wirelessRemote[0], 40);

    if((int)_keyData.btn.components.L2 == 1 && (int)_keyData.btn.components.R2 == 1){
        // Run the greenball_tracking ROS node using the system() function
        if (!running){
            std::cout << "Starting green_ball_tracking Node..."<< std::endl;
            running = true;
            pid = fork();

            if (pid == 0) {
                // This is the child process
                execl("/usr/bin/nohup", "/usr/bin/nohup", "rosrun", "greenball_tracking", "greenball_tracking", "1", (char*)NULL);
                exit(EXIT_SUCCESS);
            } else if (pid > 0) {
                // This is the parent process
                std::cout << "Child process created with PID " << pid << std::endl;
            } else {
                // fork() failed
                std::cerr << "fork() failed" << std::endl;
                exit(EXIT_FAILURE);
            }
            mqtt.setColor(0, 0, 255);
        }
        
    }
    if((int)_keyData.btn.components.L2 == 1 && (int)_keyData.btn.components.L1 == 1 && running){
        std::cout << "Stopping green_ball_tracking Node..."<< std::endl;
        mqtt.setColor(0, 0, 0);
        kill(pid, SIGTERM);
        running = false;
    }

    udp.SetSend(cmd);
}

int main(void)
{
    Joystick custom(HIGHLEVEL);
    // InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Joystick::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Joystick::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Joystick::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
