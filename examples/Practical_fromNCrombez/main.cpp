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
    // Part 1: Image acquisition

    // Part 2: Dot detection and tracking

    //Initiate the tracker
    vpDot2 dotDetector;
    //Set the graphics to true
    dotDetector.setGraphics(true);
    //check for initialization
    bool init_done = false;
    //convertion of image into image points
    vpImagePoint coordinates;

    //--------------------------------------------
    // for part 2
    bool fourPointsAcquired = false;

    // cam parameters
    int alphau = 535;
    int alphav = 535;
    int u0 = 320;
    int v0 = 240;

    vpCameraParameters camParams(alphau, alphav, u0, v0);\

    vector<vpPoint> dotObjectCoords(4);
    vector<vpImagePoint> dotImCoords(4);

    dotObjectCoords[0].setWorldCoordinates(-0.1,-0.1, 0);
    dotObjectCoords[1].setWorldCoordinates(0.1,-0.1, 0);
    dotObjectCoords[2].setWorldCoordinates(0.1,0.1, 0);
    dotObjectCoords[3].setWorldCoordinates(-0.1,0.1, 0);

    // 4 dot trackers to track 4 dots on image
    vpDot2 dot0tracker;
    dot0tracker.setGraphics(true);
    bool initd0;

    vpDot2 dot1tracker;
    dot1tracker.setGraphics(true);
    bool initd1;

    vpDot2 dot2tracker;
    dot2tracker.setGraphics(true);
    bool initd2;

    vpDot2 dot3tracker;
    dot3tracker.setGraphics(true);
    bool initd3;

    int nTrackersInitialized = 0;

    //------------------------------
    vpPose dotPatternPose;

    //while 1 for Part 1
    while(1){
        //grab the image for each lopping process
        bool videoStarted = device.getVideo(I);

        //set the display as the image just grabbed
        vpDisplay::display(I);

        //If Init is not done // click to init **** If 1
        if (! init_done) {
            if (vpDisplay::getClick(I, coordinates, false)) { // click grabber *** If 2
                dotDetector.initTracking(I, coordinates);
                init_done = true; //Init is done
            } // click grabber *** If 2
        }
        else {
            dotDetector.track(I); //Set the tracking method

        }//If Init is not done // click to init **** If 1


        // INITIALIZATON OF 4 DOT TRACKERS
        // if dot0 tracker not initialized
        if(!initd0){
            if (vpDisplay::getClick(I, coordinates, false)&& nTrackersInitialized == 0 && init_done) {
                dot0tracker.initTracking(I, coordinates);
                nTrackersInitialized++;
                initd0 = true;
            }
        }else{
            dot0tracker.track(I, dotImCoords[0]);
        }

        // if dot0 tracker not initialized
        if(!initd1){
            if (vpDisplay::getClick(I, coordinates, false) && nTrackersInitialized == 1 && initd0) {
                dot1tracker.initTracking(I, coordinates);
                nTrackersInitialized++;
                initd1 = true;
            }
        }else{
            dot1tracker.track(I, dotImCoords[1]);
        }

        // if dot0 tracker not initialized
        if(!initd2){
            if (vpDisplay::getClick(I, coordinates, false) && nTrackersInitialized == 2 && initd1) {
                dot2tracker.initTracking(I, coordinates);
                nTrackersInitialized++;
                initd2 = true;
            }
        }else{
            dot2tracker.track(I, dotImCoords[2]);
        }

        // if dot0 tracker not initialized
        if(!initd3){
            if (vpDisplay::getClick(I, coordinates, false) && nTrackersInitialized == 3 && initd2) {
                dot3tracker.initTracking(I, coordinates);
                nTrackersInitialized++;
                initd3 = true;
            }
        }else{
            dot3tracker.track(I, dotImCoords[3]);
        }


        // PART 2C : Image to sensor coordinates
        if (initd3)
        {int npoints = 4;
            for (int npoint = 0; npoint < npoints; npoint++){
                double x, y;
                vpPixelMeterConversion::convertPoint(camParams, dotImCoords[npoint], x, y);
                dotObjectCoords[npoint].set_x(x);
                dotObjectCoords[npoint].set_y(y);

                dotPatternPose.addPoint(dotObjectCoords[npoint]);

            }

            vpHomogeneousMatrix cMo;
            dotPatternPose.computePose(vpPose::LAGRANGE,cMo);
            dotPatternPose.display(I,cMo,camParams,0.05,vpColor::green);
        }
        //display
        vpDisplay::flush(I);

        if(vpDisplay::getClick(I,false) && initd3){
            break;
        }

    }//end while 1
    // End of first part
    // PART 2C : Image to sensor coordinates
    int npoints = 4;
    for (int npoint = 0; npoint < npoints; npoint++){
        double x, y;
        vpPixelMeterConversion::convertPoint(camParams, dotImCoords[npoint], x, y);
        dotObjectCoords[npoint].set_x(x);
        dotObjectCoords[npoint].set_y(y);

        dotPatternPose.addPoint(dotObjectCoords[npoint]);

    }

    vpHomogeneousMatrix cMo;
    dotPatternPose.computePose(vpPose::LAGRANGE,cMo);
    dotPatternPose.display(I,cMo,camParams,0.05,vpColor::green);


    //display
    vpDisplay::flush(I);


    //-----------------------------------------
    vpRobotCamera virtualRobot;
    virtualRobot.setPosition(cMo);

    vpServo task;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(0.5);

    vpHomogeneousMatrix cdMo(0, 0, 0.45, 0, 0, 0);
    vpFeaturePoint p[4], pd[4] ;

    for (unsigned int i = 0 ; i < 4 ; i++) {
        // project points in setup onto cam. at desired pose
        // to get desired image features (=  desired cooridnates)
        dotObjectCoords[i].track(cdMo);
        vpFeatureBuilder::create(pd[i], dotObjectCoords[i]);
        // project points in setup onto cam at current pose
        // to get current image features
        dotObjectCoords[i].track(cMo);
        vpFeatureBuilder::create(p[i], dotObjectCoords[i]);
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

    wMo = wMc * cMo;



    for (unsigned int iter = 0; iter < 1500; iter++){
        cout << "--- iter " << iter << " ------------" << endl;

        robot.getPosition(wMc);
        // get pos of cam wrt object from pos of cam wrt world
        // from cMo = cMw * wMo;
        cMo = wMc.inverse() * wMo;

        for (int i = 0; i < 4; i++){
            dotObjectCoords[i].track(cMo);
            vpFeatureBuilder::create(p[i], dotObjectCoords[i]);
        }

        vpColVector v = task.computeControlLaw();
        robot.setVelocity(vpRobot::CAMERA_FRAME, v);

        cout << cMo << endl;
        dotPatternPose.display(I,cMo,camParams,0.05,vpColor::yellow);

        //display
        vpDisplay::flush(I);
    }

    cout << "------ \n cdMo = " << cdMo << endl;
    task.kill();

    dotPatternPose.display(I,cdMo,camParams,0.05,vpColor::cyan);
    //display
    vpDisplay::flush(I);


    //*******************************************
    //*******************************************
    //*******************************************


    //device.stopVideo();



    //return 0;
}
