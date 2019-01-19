package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
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
    double x;
    double y;
    double twist;
    double throttle;
    double magnitude;
    @Override
    public void teleopInit() {
        



        
    }
    
    @Override
    public void teleopPeriodic() {
        /**Math to obtain the angle from the joystick location
         *  Then we grab the hypoteneuse for the magnitude.
         */
         x  = GamerStick.getX();
         y = -GamerStick.getY();
     //   double twist = GamerStick.getRawAxis(6);
         throttle = GamerStick.getZ();

        throttle = (throttle + 1)/2;

        x *= (throttle*3)/4;
        y *= (throttle*3)/4;
       //  twist *= (throttle*3)/4;
      //  double angle = Math.atan(x/y);
        double magnitude = Math.sqrt((y*y)+(x*x));
        if(magnitude > 0.1){
            
            //   phil.drivePolar(magnitude,angle,0.75);
            phil.driveCartesian(x,y,0);
            
     }

    }
   















    public void testPeriodic(){
         x  = GamerStick.getX();
         y = -GamerStick.getY();
         twist = GamerStick.getRawAxis(6);
         throttle = GamerStick.getZ();

        throttle = Math.abs(throttle);

        x *= (throttle*3)/4;
        y *= (throttle*3)/4;
         twist *= (throttle*3)/4;
      //  double angle = Math.atan(x/y);
        double magnitude = Math.sqrt((y*y)+(x*x));
        System.out.println("____________________________________________");
        System.out.println("X: " + x + " \nRaw X: " + GamerStick.getX());
        System.out.println("Y: " + y + " \nRaw Y: " + GamerStick.getY());
        System.out.println("Z: " + throttle);
        System.out.println("Mag: " + magnitude);
        
    }








}