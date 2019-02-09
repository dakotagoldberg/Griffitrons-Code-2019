package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.*;

public class TestCoastDrive extends TimedRobot implements Drive_Constants {
    double throttle, turn, speedL, speedR;

    WPI_TalonSRX fLeft = new WPI_TalonSRX(0);
    WPI_TalonSRX mLeft = new WPI_TalonSRX(1);
    WPI_TalonSRX bLeft = new WPI_TalonSRX(2);
    WPI_TalonSRX fRight = new WPI_TalonSRX(3);
    WPI_TalonSRX mRight = new WPI_TalonSRX(4);
    WPI_TalonSRX bRight = new WPI_TalonSRX(5);

    SpeedControllerGroup left = new SpeedControllerGroup(fLeft, mLeft, bLeft);
    SpeedControllerGroup right = new SpeedControllerGroup(fRight, mRight, fLeft);

    DifferentialDrive TestCoast = new DifferentialDrive(left, right);
    Joystick GamerStick = new Joystick(0);

    @Override
    public void autonomousInit() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {
        throttle = -GamerStick.getY();
        turn = GamerStick.getX();
        // if(Math.abs(x) > 0.05 && Math.abs(y) > 0.05){
        /*
         * if(y <= 0.1 && y >= -0.1){ speedL = x; speedR = -x; } else { if(x > 0) speedL
         * = y; speedR = y * (1-(x*3/4)); }else{ speedR = y; speedL = y *
         * (1-(Math.abs(x)*3/4)); } }
         */
        // Making turns at high speed possible
        // Need to figure out the gain value for turning
        // But when throttle = 0, the robot won't turn.
        // turn = turn * (gain_turn * Math.abs(throttle));

        // If we want to turn in place, we can add a turn button.
        // if(!turnButton)
        // turn = turn * (gain_turn * Math.abs(throttle));
        // Or we can increase the turn value when the throttle hits a certain threshold
        // if(throttle > 0.5)
        // turn = turn * (gain_turn * Math.abs(throttle));

        double t_left, t_right;

        t_left = throttle + turn;
        t_right = throttle - turn;

        speedL = t_left + skim(t_right);
        speedR = t_right + skim(t_left);

        TestCoast.tankDrive(speedL, speedR);

    }

    public void testPeriodic() {

    }

    public double skim(double v) {
        if (v > 1.0)
            return -((v - 1.0) * gain_skim);

        else if (v < -1.0)
            return -((v + 1.0) * gain_skim);

        return 0;
    }

}