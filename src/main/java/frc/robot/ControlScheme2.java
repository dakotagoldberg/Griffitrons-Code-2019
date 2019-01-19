package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;





public class ControlScheme2 extends TimedRobot {
   
    WPI_TalonSRX fLeft = new WPI_TalonSRX(0);
	WPI_TalonSRX bLeft = new WPI_TalonSRX(1);
	WPI_TalonSRX fRight = new WPI_TalonSRX(2);
	WPI_TalonSRX bRight = new WPI_TalonSRX(3);
    
    MecanumDrive phil = new MecanumDrive(fLeft, bLeft, fRight, bRight);
    Joystick GamerStick = new Joystick(0);
    
    @Override
    public void teleopInit() {
        



        
    }
    
    @Override
    public void teleopPeriodic() {
        double x = GamerStick.getX();
        double y = -GamerStick.getY();
        double twist = GamerStick.getRawAxis(6);
        double throttle = GamerStick.getZ();

        throttle = Math.abs(throttle);

        x *= (throttle*3)/4;
        y *= (throttle*3)/4;
         twist *= (throttle*3)/4;
      //  double angle = Math.atan(x/y);
        double magnitude = Math.sqrt((y*y)+(x*x));
        if(magnitude > 0.1){
            if(y<0){
            //   phil.drivePolar(magnitude,angle,0.75);
            phil.driveCartesian(x,y,0);
            }
     }

    }
   















    public void testPeriodic(){
        System.out.println("X: " + GamerStick.getX());
        System.out.println("Y: " + GamerStick.getY());
        System.out.println("");
    }








}