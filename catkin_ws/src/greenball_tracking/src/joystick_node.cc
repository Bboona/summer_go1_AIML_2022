/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <unitree_legged_sdk/unitree_legged_sdk.h>
#include <unitree_legged_sdk/joystick.h>
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <signal.h>

using namespace UNITREE_LEGGED_SDK;

class Joystick
{
public:
    Joystick(uint8_t level): 
        safe(LeggedType::Go1), 
        udp(level, 8090, "192.168.123.161", 8082){
        udp.InitCmdData(cmd);
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

    if((int)_keyData.btn.components.down == 1){
        std::cout << "The key down is pressed"<< std::endl;
        std::cout << "Running MQTT command"<< std::endl;
        int child_pid = fork();
        if (child_pid == 0) {
            // This is the child process, so run the second package
            execlp("python3", "python3", "../change_LED_red_mqtt.py", NULL);
            // If execlp returns, it has failed
            std::cerr << "Failed to start second package!" << std::endl;
            exit(1);
        }

        // Wait for the user to press enter, then kill the second package
        std::cout << "Press enter to terminate second package..." << std::endl;
        std::cin.get();
        kill(child_pid, SIGKILL);
    }
    if((int)_keyData.btn.components.right == 1){
        std::cout << "The key right is pressed"<< std::endl;
        std::cout << "Running MQTT command"<< std::endl;
        int child_pid = fork();
        if (child_pid == 0) {
            // This is the child process, so run the second package
            execlp("python3", "python3", "../change_LED_blue_mqtt.py", NULL);
            // If execlp returns, it has failed
            std::cerr << "Failed to start second package!" << std::endl;
            exit(1);
        }

        // Wait for the user to press enter, then kill the second package
        std::cout << "Press enter to terminate second package..." << std::endl;
        std::cin.get();
        kill(child_pid, SIGKILL);
    } 
    if((int)_keyData.btn.components.left == 1){
        std::cout << "The key left is pressed"<< std::endl;
        std::cout << "Running MQTT command"<< std::endl;
        int child_pid = fork();
        if (child_pid == 0) {
            // This is the child process, so run the second package
            execlp("python3", "python3", "../change_LED_green_mqtt.py", NULL);
            // If execlp returns, it has failed
            std::cerr << "Failed to start second package!" << std::endl;
            exit(1);
        }

        // Wait for the user to press enter, then kill the second package
        std::cout << "Press enter to terminate second package..." << std::endl;
        std::cin.get();
        kill(child_pid, SIGKILL);
    }
    if((int)_keyData.btn.components.up == 1){
        std::cout << "The key up is pressed"<< std::endl;
        std::cout << "Running MQTT command"<< std::endl;
        int child_pid = fork();
        if (child_pid == 0) {
            // This is the child process, so run the second package
            execlp("python3", "python3", "../turnoff_LED_mqtt.py", NULL);
            // If execlp returns, it has failed
            std::cerr << "Failed to start second package!" << std::endl;
            exit(1);
        }

        // Wait for the user to press enter, then kill the second package
        std::cout << "Press enter to terminate second package..." << std::endl;
        std::cin.get();
        kill(child_pid, SIGKILL);
    } 

    udp.SetSend(cmd);
}

int main(void)
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

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
