#include <iostream>
#include <signal.h>
#include <cmath>
#include <cerrno>
#include <cfenv>
#include <ros/ros.h>

#include <sensor_msgs/Range.h>



#include <UnitreeCameraSDK.hpp>
#include <unitree_legged_sdk/unitree_legged_sdk.h>



#include <unistd.h>
#include <iostream>
#include <deque>

using namespace std;
using namespace cv;

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level): 
      safe(LeggedType::Go1), 
      udp(level, 8090, "192.168.123.161", 8082){
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RightRobotControl();
    void LeftRobotControl();
    void ForwardRobotControl();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
};


void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::LeftRobotControl() 
{
    motiontime += 2;
    udp.GetRecv(state);
    printf("%d   %f\n", motiontime, state.imu.quaternion[2]);

    cmd.mode = 0;      // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0]  = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    cmd.reserve = 0;

    
    cmd.mode = 2;
    cmd.gaitType = 1;
    cmd.velocity[0] = 0.05f; // -1  ~ +1
    cmd.yawSpeed = 0.5;
    cmd.footRaiseHeight = 0.1;
    // printf("walk\n");
    

    udp.SetSend(cmd);
}

void Custom::RightRobotControl() 
{
    motiontime += 2;
    udp.GetRecv(state);
    printf("%d   %f\n", motiontime, state.imu.quaternion[2]);

    cmd.mode = 0;      // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0]  = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    cmd.reserve = 0;

    
    cmd.mode = 2;
    cmd.gaitType = 1;
    cmd.velocity[0] = 0.05f; // -1  ~ +1
    cmd.yawSpeed = -0.5;
    cmd.footRaiseHeight = 0.1;
    // printf("walk\n");
    

    udp.SetSend(cmd);
}

void Custom::ForwardRobotControl() 
{
    motiontime += 2;
    udp.GetRecv(state);
    printf("%d   %f\n", motiontime, state.imu.quaternion[2]);

    cmd.mode = 0;      // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0]  = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.15f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    cmd.reserve = 0;

    
    cmd.mode = 2;
    cmd.gaitType = 1;
    cmd.velocity[0] = 0.1f; // -1  ~ +1
    cmd.yawSpeed = 0.0;
    cmd.footRaiseHeight = 0.1;
    // printf("walk\n");
    

    udp.SetSend(cmd);
}



int main(int argc, char *argv[]){

    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(HIGHLEVEL);
    
    int deviceNode = 0; ///< default 0 -> /dev/video0
    cv::Size frameSize(1856, 800); ///< default frame size 1856x800
    int fps = 30; ///< default camera fps: 30
    
    if(argc >= 2){
        deviceNode = std::atoi(argv[1]);
        if(argc >= 4){
            frameSize = cv::Size(std::atoi(argv[2]), std::atoi(argv[3]));
        }
        if(argc >=5)
            fps = std::atoi(argv[4]);
    }
    
    UnitreeCamera cam(deviceNode); ///< init camera by device node number
    if(!cam.isOpened())   ///< get camera open state
        exit(EXIT_FAILURE);
    
    cam.setRawFrameSize(frameSize); ///< set camera frame size
    cam.setRawFrameRate(fps);       ///< set camera camera fps
    cam.setRectFrameSize(cv::Size(frameSize.width >> 2, frameSize.height >> 1)); ///< set camera rectify frame size
    cam.startCapture(); ///< disable image h264 encoding and share memory sharing

    // define the lower and upper boundaries of the "green"
    // ball in the HSV color space
    // Scalar greenLower(29, 86, 6);
    // Scalar greenUpper(64, 255, 255);

    Scalar greenLower(29, 117, 6);
    Scalar greenUpper(105, 255, 255);

    // Scalar greenLower(25, 70, 0);
    // Scalar greenUpper(64, 255, 255);

    // Scalar greenLower(29, 87, 30);
    // Scalar greenUpper(84, 125, 150);

    int bufferSize = 64;

    string detected_path = "/home/unitree/robot_camera_output/green_ball/detected/";
    string not_detected_path = "/home/unitree/robot_camera_output/green_ball/not-detected/";

    deque<Point> pts;
    pts.resize(bufferSize);

    int d = 0;
    int n = 0;
    
    usleep(500000);
    while(cam.isOpened()){
        cv::Mat left,right,feim;
        if(!cam.getRectStereoFrame(left,right,feim)){ ///< get longlat rectify left,right and fisheye rectify feim  
            usleep(1000);
            continue;
        }
        
        cv::Mat stereo;
        cv::Mat blurred;
        cv::Mat hsv;

        cv::flip(feim,feim,0);
        cv::flip(feim,feim,1);
        cv::flip(left,left,0);
        cv::flip(left,left,1);
        cv::flip(right,right,0);
        cv::flip(right,right,1);
        cv::hconcat(left,right, stereo);
	    cv::Mat original = feim.clone();

        // blur it, and convert it to the HSV color space
        GaussianBlur(feim, blurred, Size(11, 11), 0);
        cvtColor(blurred, hsv, COLOR_BGR2HSV);

        // construct a mask for the color "green", then perform
        // a series of dilations and erosions to remove any small
        // blobs left in the mask
        Mat mask;
        Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
        inRange(hsv, greenLower, greenUpper, mask);
        erode(mask, mask, Mat(), Point(-1, -1), 2);
        dilate(mask, mask, kernel, Point(-1, -1), 2);

        // find contours in the mask and initialize the current
        // (x, y) center of the ball
        vector<vector<Point> > contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        //drawContours(feim, contours, -1, Scalar(0, 255, 0), 2);
        
        Point2f center;
        float radius = 0;
        if (!contours.empty())
        {
            // find the largest contour in the mask, then use it to compute the
            // minimum enclosing circle and centroid
            int idx = 0;
            for (int i = 1; i < contours.size(); i++)
            {
                if (contourArea(contours[i]) > contourArea(contours[idx]))
                    idx = i;
            }
	
            minEnclosingCircle(contours[idx], center, radius);
            Moments M = moments(contours[idx]);
            center.x = int(M.m10 / M.m00);
            center.y = int(M.m01 / M.m00);

            // only proceed if the radius meets a minimum size
            if (radius > 3)
            {
                // draw the circle and centroid on the frame,
                // then update the list of tracked points
		        string file_path = detected_path + to_string(d) + "_raw.png";
		        //imwrite(file_path,original);
                circle(feim, Point(int(center.x), int(center.y)), int(radius), Scalar(0, 255, 255), 1);
                //circle(feim, center, 5, Scalar(0, 0, 255), -1);
                file_path = detected_path + to_string(d) + ".png";
		        //imwrite(file_path,feim);
		        //d++;
                if (center.x > 310){
                    cout<<center.x<<" "<<center.y<<" turning right"<<endl;
                    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RightRobotControl, &custom));
                    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
                    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

                    loop_udpSend.start();
                    loop_udpRecv.start();
                    loop_control.start();
                }else if (center.x < 90){
                    cout<<center.x<<" "<<center.y<<" turning left"<<endl;
                    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::LeftRobotControl, &custom));
                    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
                    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

                    loop_udpSend.start();
                    loop_udpRecv.start();
                    loop_control.start();
                }
                else{
                    cout<<center.x<<" "<<center.y<<" turning left"<<endl;
                    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::ForwardRobotControl, &custom));
                    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
                    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

                    loop_udpSend.start();
                    loop_udpRecv.start();
                    loop_control.start();
                }
                
            }
        }
        else{

            //string file_path = not_detected_path + to_string(n) + ".png";
            //imwrite(file_path,original);
            //n++;
        }

        // update the points queue
        pts.push_front(center);

        // // loop over the set of tracked points
        // for (int i = 1; i < pts.size(); i++)
        // {
        //     // if either of the tracked points are None, ignore
        //     // them
        //     if (pts[i - 1] == Point(0, 0) || pts[i] == Point(0, 0))
        //         continue;

        //     // otherwise, compute the thickness of the line and
        //     // draw the connecting lines
        //     int thickness = int(sqrt(bufferSize / float(i + 1)) * 2.5);
        //     line(feim, pts[i - 1], pts[i], (0, 0, 255), thickness);
        // }


        //cv::imshow("Mask",mask);
        cv::imshow("FishEye_Rect", feim);
        //cv::imshow("Longlat_Rect", stereo);

        char key = cv::waitKey(10);
        if(key == 27) // press ESC key
           break;
    }
    
    cam.stopCapture(); ///< stop camera capturing
    
    return 0;
}

