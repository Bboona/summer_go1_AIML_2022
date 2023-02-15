#include <iostream>
#include <signal.h>
#include <cmath>
#include <cerrno>
#include <cfenv>
#include <ros/ros.h>
#include "std_srvs/Trigger.h"
#include "geometry_msgs/Point.h"
#include <UnitreeCameraSDK.hpp>
#include <unistd.h>
#include <iostream>
#include <deque>

using namespace std;
using namespace cv;


int main(int argc, char *argv[]){

    ros::init(argc, argv, "green_ball");
    ros::NodeHandle nh;

    // Create a publisher to publish the ball points
    ros::Publisher point_pub = nh.advertise<geometry_msgs::Point>("ball_coordinates", 10);

    // Variable which checks if joystick combinations have been pressed
    bool start_detection = false;

    // Publish ball points at a rate of 10 Hz
    ros::Rate rate(10);

    while(ros::ok()) {
        
        // Check rosparam current value to see if Point message is required 
        nh.getParam("/start_detection", start_detection);

        if(start_detection) {
            int deviceNode = 1; // default 1 -> /dev/video1 -> face camera
            cv::Size frameSize(1856, 800); ///< default frame size 1856x800
            int fps = 30; ///< default camera fps: 30
        
            // Can change camera configurations with arguments passed to main
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
            Scalar greenLower(29, 117, 6);
            Scalar greenUpper(105, 255, 255);

            int bufferSize = 64;

            deque<Point> pts;
            pts.resize(bufferSize);


            usleep(500000);
            while(cam.isOpened() && ros::ok() && start_detection){
                
                // Check rosparam current value to see if Point message is required 
                nh.getParam("/start_detection", start_detection);

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
                    if (radius > 5)
                    {
                        // draw the circle and centroid on the frame,
                        // then update the list of tracked points
                        // circle(feim, Point(int(center.x), int(center.y)), int(radius), Scalar(0, 255, 255), 1);
                        // circle(feim, center, 5, Scalar(0, 0, 255), -1);

                        // Define the point message and fill it with the values of x and y
                        geometry_msgs::Point point;
                        point.x = center.x;  // Set x to 1.0
                        point.y = center.y;  // Set y to 2.0
                        point.z = radius;

                        // Publish the point message
                        point_pub.publish(point);

                        ////////////////////////////////////////////////////

                        ////// CAN PUBSLISH MORE INFORMATION HERE //////////
                        
                        ////////////////////////////////////////////////////


                    }
                }

                // Sleep for the desired rate
                rate.sleep();
            }
            std::cout << "Closing the camera"<< std::endl;
            cam.stopCapture(); ///< stop camera capturing
        }
        rate.sleep();
    }
    // Keep running until the node is stopped
    ros::spin();
    return 0;
}

