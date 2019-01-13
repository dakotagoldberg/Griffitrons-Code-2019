package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;





public class ControlScheme2 extends TimedRobot {
   
    WPI_TalonSRX fLeft = new WPI_TalonSRX(0);
	WPI_TalonSRX bLeft = new WPI_TalonSRX(1);
	WPI_TalonSRX fRight = new WPI_TalonSRX(2);
	WPI_TalonSRX bRight = new WPI_TalonSRX(3);
    
    Joystick GamerStick = new Joystick(0);
    
    @Override
    public void teleopInit() {

        


        
    }
    
    @Override
    public void teleopPeriodic() {
<<<<<<< HEAD
        double angle = Math.atan(GamerStick.getX()/GamerStick.getY());
        double magnitude = Math.sqrt((GamerStick.getX()*GamerStick.getX())/(Math.pow(GamerStick.getY));
        
        
=======

>>>>>>> 5cd9d98998545d9a64d449e44048e20446b385c7
    }















    public void testPeriodic(){
        System.out.println("X: " + GamerStick.getX());
        System.out.println("Y: " + GamerStick.getY());
    }








}