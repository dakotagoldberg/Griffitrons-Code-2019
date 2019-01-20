package frc.robot;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TestCoastDrive extends TimedRobot{
    double x,y,speedL,speedR;
    
    WPI_TalonSRX fLeft = new WPI_TalonSRX(0);
	WPI_TalonSRX bLeft = new WPI_TalonSRX(1);
	WPI_TalonSRX fRight = new WPI_TalonSRX(2);
    WPI_TalonSRX bRight = new WPI_TalonSRX(3);
    
    SpeedControllerGroup left = new SpeedControllerGroup(fLeft,bLeft);
    SpeedControllerGroup right = new SpeedControllerGroup(fRight,fLeft);

    DifferentialDrive TestCoast = new DifferentialDrive(left,right);
    Joystick GamerStick = new Joystick(0);
    @Override
    public void teleopInit(){

    }
    @Override
    public void teleopPeriodic() {
    x = GamerStick.getX();
    y = -GamerStick.getY();
    if(Math.abs(x) > 0.05 && Math.abs(y) > 0.05){
        if(y <= 0.1 && y >= -0.1){
            speedL = x;
            speedR = -x;
        } else {
            if(x > 0){
              if(x < 0.05 && x > -0.05)
                x = 0;
              speedL = y;
              speedR = y * (1-(x*3/4));
            }else{
                if(x < 0.05 && x > -0.05)
                    x = 0;
                speedR = y;
                speedL = y * (1-(Math.abs(x)*3/4));
            }
        }
    }
    TestCoast.tankDrive(speedL,speedR);
    }
    public void testPeriodic(){


    }








}