/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.possibleclasses;


public class Vision {
    enum States{
        Finding,Found, LinedUp;
    }
    int CenterX, CenterY,CenterTheta;
    //udp listener


    public Vision(){
        States s = States.Finding;
    }

    private boolean cameraFeed(){
        //whether or not you want camera feed
        return true;
    }
    private void CameraFeed(){
        //gives the camera feed, obviously the return type isn't just Camera feed
    }
    public boolean acquired(){
        //returns whether or not its on target allready
        return true;
    }
    public int[] getGoal(){
        //calculates cordinates needed for the tape and feeds it to drive
        return null;
    }
    public void getCameraFeed(int cameraNum){
        //gets camera feed, obviously not returning type camerafeed
    }

}
