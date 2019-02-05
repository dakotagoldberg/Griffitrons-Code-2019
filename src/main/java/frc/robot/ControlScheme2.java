package frc.robot;
//import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


// @authors: Spencer Collins, Dakota Goldberg, and Leonard Kakinuma

public class ControlScheme2 extends TimedRobot {
   
    WPI_TalonSRX fLeft = new WPI_TalonSRX(0);
	WPI_TalonSRX bLeft = new WPI_TalonSRX(1);
	WPI_TalonSRX fRight = new WPI_TalonSRX(2);
	WPI_TalonSRX bRight = new WPI_TalonSRX(3);
    
    MecanumDrive phil = new MecanumDrive(fLeft, bLeft, fRight, bRight);
    Joystick GamerStick = new Joystick(0);
    //all the variables.
    double x,y,throttle,magnitude;
    boolean left, right,ram;

    //test variables
    int gX = 4,gY = 4;
    @Override
    public void teleopInit() {
         
    }
    
    @Override
    public void teleopPeriodic() {
        //gets the button inputs and assignes them, y inverted
        ram = GamerStick.getRawButton(1);
        left = GamerStick.getRawButton(4);
        right = GamerStick.getRawButton(5);
        x  = GamerStick.getX();
        y = -GamerStick.getY();
        throttle = GamerStick.getZ();
        //math
        throttle = (-throttle + 1)/2;
        x *= (throttle*3)/4;
        y *= (throttle*3)/4;
        magnitude = Math.sqrt((y*y)+(x*x));
        //checks for left right buttons
        if(left && !right){
            phil.driveCartesian(0,0,-0.3);
        } 
        if(right && !left){
            phil.driveCartesian(0,0,0.3);
        }
        //dead zone
        if(magnitude > 0.05){
            phil.driveCartesian(x,y,0);
     }

    }
  //  public void testInit(){
   //     gX = 4;
  //      gY = 4;
   // }

    public void testPeriodic(){
        if(GamerStick.getRawButton(6)){
        x  = GamerStick.getX();
        y = -GamerStick.getY();
        
        throttle = GamerStick.getZ();

        throttle = Math.abs(throttle);

        x *= (throttle*3)/4;
        y *= (throttle*3)/4;
        
         left = GamerStick.getRawButton(4);
         right = GamerStick.getRawButton(5);
         
        double magnitude = Math.sqrt((y*y)+(x*x));
        System.out.println("----------------------------------------------------------58");
        System.out.println("X: " + x + " \nRaw X: " + GamerStick.getX());
        System.out.println("Y: " + y + " \nRaw Y: " + GamerStick.getY());
        System.out.println("Z: " + throttle);
        System.out.println("Mag: " + magnitude);
        System.out.println("Left: " + left + " Right: " + right);
        } else {
        //don't worry 'bout this
        if(gX < 1)
            gX = 1;
        if(gX > 8)
            gX = 8;
        if(gY < 1)
            gY = 1;
        if(gY < 8)
            gY = 8;
        
        if(GamerStick.getRawButton(3))
            gY -= 1;
        if(GamerStick.getRawButton(2))
            gY += 1;
        if(GamerStick.getRawButton(4))
            gX -= 1;
        if(GamerStick.getRawButton(5))
            gX += 1;
        
            for(int y = 0; y < 10; y++){
                for(int x = 0; x < 10; x++){
                    if(y == 0 || y == 9){
                        System.out.print('-');
                    } else if(x == 0 || x == 9){
                        System.out.print('|');
                    }else if(x== gX && y == gY){
                        System.out.print('0');
                    } else {
                        System.out.print('/');
                    }
                }
                System.out.println();
            }
        }
    }



}