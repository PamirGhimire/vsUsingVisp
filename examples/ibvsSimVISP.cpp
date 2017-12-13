// source :
// http://visp-doc.inria.fr/doxygen/visp-2.8.0/tutorial-ibvs-4pts_8cpp_source.html

// Pamir Ghimire, December 13, 2017

#include <visp/vpFeatureBuilder.h>
#include <visp/vpServo.h>
#include <visp/vpSimulatorCamera.h>

#include<iostream>
using namespace std;

int main(){

    // WORLD SETUP:
    // UNKNOWN TO THE VISUAL SERVOING TASK:
    //-----------------------------------------
    // desired object to camera
    vpHomogeneousMatrix cdMo(0, 0, 0.75, 0, 0, 0);

    // initial object to camera
    vpHomogeneousMatrix cMo(0.15, -0.1, 1.,
                            vpMath::rad(10), vpMath::rad(-10), vpMath::rad(50));

    // 4 points defined in the world, to be projected on cam.
    vpPoint point[4];
    point[0].setWorldCoordinates(-0.1,-0.1, 0);
    point[1].setWorldCoordinates( 0.1,-0.1, 0);
    point[2].setWorldCoordinates( 0.1, 0.1, 0);
    point[3].setWorldCoordinates(-0.1, 0.1, 0);
    //-----------------------------------------

    // vs task: produces velocities using error in image features
    vpServo task ;
    // robot-cam config:
    task.setServo(vpServo::EYEINHAND_CAMERA);

    // -lambda x dot(e) = L.Vc, L= interaction matrix
    // possible choices : current, desired, mean, user_defined
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(0.5);

    // current and desired features, p and pd
    vpFeaturePoint p[4], pd[4];
    for (int i = 0; i < 4; i++){
        // project points in setup onto cam. at desired pose
        // to get desired image features (=  desired cooridnates)
        point[i].track(cdMo);
        vpFeatureBuilder::create(pd[i], point[i]);

        // project points in setup onto cam at current pose
        // to get current image features
        point[i].track(cMo);
        vpFeatureBuilder::create(p[i], point[i]);

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
            point[i].track(cMo);
            vpFeatureBuilder::create(p[i], point[i]);
        }

        vpColVector v = task.computeControlLaw();
        robot.setVelocity(vpRobot::CAMERA_FRAME, v);

        cout << cMo << endl;
    }

    cout << "------ \n cdMo = " << cdMo << endl;
    task.kill();

    return 0;
}
