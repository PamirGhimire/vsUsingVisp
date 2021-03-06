#include "MyFreenectDevice.hpp"
#include <iostream>
#include <algorithm>
#include <vector>


#include <visp/vpImageIo.h>
#include <visp/vpIoTools.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>
#include <visp/vpPoint.h>
#include <visp/vpPose.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpServo.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpSimulatorCamera.h>




using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    //instanciate the image grabber
    vpImage<unsigned char> I(480,640);
    //instanciate the display
    vpDisplayX d(I);
    //create a kinect object
    Freenect::Freenect freenect;
    //Instanciate the kinect grabber
    MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);
    device.startVideo();

    //*******************************************
    //*************TODO by STUDENTS**************
    //*******************************************
    //Initialize the tracker for big dot in the center
    vpDot2 dotDetector;

    //Enable display of previous tracker's track
    dotDetector.setGraphics(true);

    //Initialization of the central dot tracker
    bool init_done = false;

    //Track of the central dot tracker (in image frame)
    vpImagePoint coordinates;

    // Apriori:
    // Camera Intrinsics, used later for pose estimation
    int alphau = 535;
    int alphav = 535;
    int u0 = 320;
    int v0 = 240;

    // camera-parameters object using known intrinsics
    vpCameraParameters camParams(alphau, alphav, u0, v0);\

    // coordinates of dots in object and sensor frames (unit = meters)
    vector<vpPoint> dotCoords(4);
    // coordinates of dots in image frame (unit = pixels)
    vector<vpImagePoint> dotImCoords(4);

    // known object-frame coordinates of four corner dots in pattern
    dotCoords[0].setWorldCoordinates(-0.1,-0.1, 0);
    dotCoords[1].setWorldCoordinates(0.1,-0.1, 0);
    dotCoords[2].setWorldCoordinates(0.1,0.1, 0);
    dotCoords[3].setWorldCoordinates(-0.1,0.1, 0);

    // 4 dot trackers to track 4 dots on image
    vpDot2 dot0tracker;
    dot0tracker.setGraphics(true);
    bool initd0 = false;

    vpDot2 dot1tracker;
    dot1tracker.setGraphics(true);
    bool initd1 = false;

    vpDot2 dot2tracker;
    dot2tracker.setGraphics(true);
    bool initd2 = false;

    vpDot2 dot3tracker;
    dot3tracker.setGraphics(true);
    bool initd3 = false;

    int nTrackersInitialized = 0;

    //------------------------------
    // Pose estimation object to estimate pose of 4-dots pattern
    vpPose dotPatternPose;
    // relative pose between pattern and camera
    vpHomogeneousMatrix cMo;


    //while 1 for Part 1
    while(1){
        // grab an image from the video device
        bool videoStarted = device.getVideo(I);

        //set at the display the image just grabbed
        vpDisplay::display(I);

        //If central dot-tracker not initialized
        if (! init_done) {
            // get image coordinates of mouse click in display-window
            if (vpDisplay::getClick(I, coordinates, false)) {
                // initialize central-dot tracker at location of latest click
                dotDetector.initTracking(I, coordinates);
                init_done = true;
            }
        }
        // If central dot-tracker initialized
        else {
            // track the central dot
            dotDetector.track(I);

        }


        // 4 DOT TRACKERS:
        //---------------------------------
        // if dot0 tracker not initialized
        if(!initd0){
            // if click on display window and central dot-tracker is already initialized
            if (vpDisplay::getClick(I, coordinates, false)&& nTrackersInitialized == 0 && init_done) {
                // initialize dot0 tracker with location of latest click
                dot0tracker.initTracking(I, coordinates);
                nTrackersInitialized++;
                initd0 = true;
            }
        }
        // if dot0-tracker is initialized
        else{
            // track it
            dot0tracker.track(I, dotImCoords[0]);
        }

        //---------------------------------
        // if dot1 tracker not initialized
        if(!initd1){
            // if click on display window and dot0-tracker is already initialized
            if (vpDisplay::getClick(I, coordinates, false) && nTrackersInitialized == 1 && initd0) {
                // initialize dot1 tracker with location of latest click
                dot1tracker.initTracking(I, coordinates);
                nTrackersInitialized++;
                initd1 = true;
            }
        }else{
            dot1tracker.track(I, dotImCoords[1]);
        }

        // if dot2 tracker not initialized
        if(!initd2){
            if (vpDisplay::getClick(I, coordinates, false) && nTrackersInitialized == 2 && initd1) {
                dot2tracker.initTracking(I, coordinates);
                nTrackersInitialized++;
                initd2 = true;
            }
        }else{
            dot2tracker.track(I, dotImCoords[2]);
        }

        // if dot3 tracker not initialized
        if(!initd3){
            if (vpDisplay::getClick(I, coordinates, false) && nTrackersInitialized == 3 && initd2) {
                dot3tracker.initTracking(I, coordinates);
                nTrackersInitialized++;
                initd3 = true;
            }
        }else{
            dot3tracker.track(I, dotImCoords[3]);
        }


        // If all 4 dot-trackers have been initialized
        if (initd3){
            int npoints = 4;
            // convert image coordinates of all 4 dots to sensor coordinates
            for (int npoint = 0; npoint < npoints; npoint++){
                double x, y;
                vpPixelMeterConversion::convertPoint(camParams, dotImCoords[npoint], x, y);
                dotCoords[npoint].set_x(x);
                dotCoords[npoint].set_y(y);

                dotPatternPose.addPoint(dotCoords[npoint]);
            }

            dotPatternPose.computePose(vpPose::LAGRANGE,cMo);
            dotPatternPose.display(I,cMo,camParams,0.05,vpColor::green);
        }
        //display
        vpDisplay::flush(I);

        if(vpDisplay::getClick(I,false) && initd3){
            break;
        }

    }//end while 1


    // convert image coordinates of 4 dots in the latest tracked frame
    // into sensor coordinates
    int npoints = 4;
    for (int npoint = 0; npoint < npoints; npoint++){
        double x, y;
        vpPixelMeterConversion::convertPoint(camParams, dotImCoords[npoint], x, y);
        dotCoords[npoint].set_x(x);
        dotCoords[npoint].set_y(y);

        dotPatternPose.addPoint(dotCoords[npoint]);
    }

    // compute the relative pose between cam and pattern in latest tracked frame
    dotPatternPose.computePose(vpPose::LAGRANGE,cMo);
    // visualize the relative pose of the pattern
    dotPatternPose.display(I,cMo,camParams,0.05,vpColor::green);
    //display
    vpDisplay::flush(I);

    //-----------------------------------------
    // VIRTUAL VISUAL SERVOING (POINTS BASED):
    //-----------------------------------------
    // virtual cam. that will be visually servoed
    vpRobotCamera virtualRobot;
    // initialize its pose wrt object at pose of cam. in last tracked frame
    virtualRobot.setPosition(cMo);

    // visual servo task object handles all computations related to visual servoing
    vpServo task;
    // eye-in-hand configuration
    task.setServo(vpServo::EYEINHAND_CAMERA);
    // interaction matrix computed from current features
    task.setInteractionMatrixType(vpServo::CURRENT);
    // gain
    task.setLambda(0.5);

    // arbitrarily chosen desired pose of camera wrt object
    vpHomogeneousMatrix cdMo(0, 0, 0.45, 0, 0, 0);
    // arrays for current and desired image features (dot coordinates)
    vpFeaturePoint p[4], pd[4];

    for (unsigned int i = 0 ; i < 4 ; i++) {
        // project points in setup onto cam. at desired pose
        // to get desired image features (=  desired cooridnates)
        dotCoords[i].track(cdMo);
        vpFeatureBuilder::create(pd[i], dotCoords[i]);
        // project points in setup onto cam at current pose
        // to get current image features
        dotCoords[i].track(cMo);
        vpFeatureBuilder::create(p[i], dotCoords[i]);
        // add features to task
        task.addFeature(p[i], pd[i]);
    }

    // we introduce a world frame here
    vpHomogeneousMatrix wMc, wMo;
    // the robot is going to move in the world frame
    vpSimulatorCamera robot;
    // frequency at which the world is observed (?)
    robot.setSamplingTime(0.040);
    // get current pose of the robot in the world
    robot.getPosition(wMc);

    // get object2world from cam2world since object2cam is known
    wMo = wMc * cMo;

    // VISUAL SERVOING:
    for (unsigned int iter = 0; iter < 1500; iter++){
        // print current iteration
        cout << "--- iter " << iter << " ------------" << endl;

        robot.getPosition(wMc);
        // get pos of cam wrt object from pos of cam wrt world
        // from cMo = cMw * wMo;
        cMo = wMc.inverse() * wMo;

        for (int i = 0; i < 4; i++){
            dotCoords[i].track(cMo);
            vpFeatureBuilder::create(p[i], dotCoords[i]);
        }

        vpColVector v = task.computeControlLaw();
        robot.setVelocity(vpRobot::CAMERA_FRAME, v);

        cout << cMo << endl;
        dotPatternPose.display(I,cMo,camParams,0.05,vpColor::yellow);

        //display
        vpDisplay::flush(I);
    }

    // compare final cMo with cdMo
    cout << "------ \n cdMo = " << cdMo << endl;
    task.kill();

    // display the would-be estimate of pattern pose at cdMo
    dotPatternPose.display(I,cdMo,camParams,0.05,vpColor::cyan);
    //display
    vpDisplay::flush(I);


    //*******************************************
    //*******************************************
    //*******************************************


    //device.stopVideo();



    //return 0;
}
