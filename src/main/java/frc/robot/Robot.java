package frc.robot;

import frc.robot.auto.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.auto.EchoServer;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Robot extends TimedRobot implements Drive_Constants, Control_Constants {
    double x, y, throttle, turn, speedL, speedR;

    // WPI_TalonSRX fLeft = new WPI_TalonSRX(front_left);
    // WPI_TalonSRX mLeft = new WPI_TalonSRX(middle_left);
    // WPI_TalonSRX bLeft = new WPI_TalonSRX(back_left);
    // WPI_TalonSRX fRight = new WPI_TalonSRX(front_right);
    // WPI_TalonSRX mRight = new WPI_TalonSRX(middle_right);
    // WPI_TalonSRX bRight = new WPI_TalonSRX(back_right);

    // EchoServer jetson;
    Intake claws;

    // SpeedControllerGroup left = new SpeedControllerGroup(fLeft, mLeft, bLeft);
    // SpeedControllerGroup right = new SpeedControllerGroup(fRight, mRight, bRight);

    // DifferentialDrive TestCoast = new DifferentialDrive(left, right);
    XboxController driveBox = new XboxController(drive_controller);

    // DoubleSolenoid leftSole = new DoubleSolenoid(0, 1);
    // DoubleSolenoid rightSole = new DoubleSolenoid(2, 3);

    @Override
    public void robotInit() {
        // jetson = new EchoServer();
        claws = new Intake();

        // fLeft.configContinuousCurrentLimit(continuous_current);
        // mLeft.configContinuousCurrentLimit(continuous_current);
        // bLeft.configContinuousCurrentLimit(continuous_current);
        // fRight.configContinuousCurrentLimit(continuous_current);
        // mRight.configContinuousCurrentLimit(continuous_current);
        // bRight.configContinuousCurrentLimit(continuous_current);
        // fLeft.configPeakCurrentLimit(peak_current);
        // mLeft.configPeakCurrentLimit(peak_current);
        // bLeft.configPeakCurrentLimit(peak_current);
        // fRight.configPeakCurrentLimit(peak_current);
        // mRight.configPeakCurrentLimit(peak_current);
        // bRight.configPeakCurrentLimit(peak_current);
        // fLeft.configOpenloopRamp(open_ramp);
        // mLeft.configOpenloopRamp(open_ramp);
        // bLeft.configOpenloopRamp(open_ramp);
        // fRight.configOpenloopRamp(open_ramp);
        // mRight.configOpenloopRamp(open_ramp);
        // bRight.configOpenloopRamp(open_ramp);
    }

    @Override
    public void robotPeriodic() {

    }

    @Override
    public void autonomousInit() {
       // jetson.start();
    }

    @Override
    public void autonomousPeriodic() {
        // throttle = jetson.getThrottle();
        // speedL = throttle;
        // speedR = throttle;
        // // System.out.println(speedL + " " + speedR);
        // TestCoast.tankDrive(speedL, speedR);
    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {
        // y = -driveBox.getRawAxis(left_y_axis);
        // x = driveBox.getRawAxis(right_x_axis);

        // if (Math.abs(y) > 0.01)
        //     throttle = y;
        // else
        //     throttle = 0.0;

        // if (Math.abs(x) > 0.25)
        //     turn = x;
        // else
        //     turn = 0.0;

        // // if(Math.abs(x) > 0.05 && Math.abs(y) > 0.05){
        // /*
        //  * if(y <= 0.1 && y >= -0.1){ speedL = x; speedR = -x; } else { if(x > 0) speedL
        //  * = y; speedR = y * (1-(x*3/4)); }else{ speedR = y; speedL = y *
        //  * (1-(Math.abs(x)*3/4)); } }
        //  */
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

        // double t_left, t_right;
        // t_left = throttle + turn;
        // t_right = throttle - turn;

        // speedL = t_left + skim(t_right);
        // speedR = t_right + skim(t_left);

        // TestCoast.tankDrive(speedL, speedR);

        // if (driveBox.getRawButton(left_bumper)) {
        //     leftSole.set(DoubleSolenoid.Value.kReverse);
        //     rightSole.set(DoubleSolenoid.Value.kReverse);
        // } else if (driveBox.getRawButton(right_bumper)) {
        //     leftSole.set(DoubleSolenoid.Value.kForward);
        //     rightSole.set(DoubleSolenoid.Value.kForward);
        // } else {
        //     leftSole.set(DoubleSolenoid.Value.kOff);
        //     rightSole.set(DoubleSolenoid.Value.kOff);
        // }

       

    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    if(driveBox.getRawButton(4))
        claws.hatchIntake();
    else if(driveBox.getRawButton(5))
        claws.hatchShoot();
    //     if (driveBox.getRawButton(left_bumper)) {
    //         fLeft.set(0);
    //         mLeft.set(0);
    //         mLeft.set(0);
    //         if (driveBox.getRawButton(x_button)) {
    //             fRight.set(.5);
    //             mRight.set(0);
    //             bRight.set(0);
    //         } else if (driveBox.getRawButton(y_button)) {
    //             fRight.set(0);
    //             mRight.set(.5);
    //             bRight.set(0);
    //         } else if (driveBox.getRawButton(b_button)) {
    //             fRight.set(0);
    //             mRight.set(0);
    //             bRight.set(.5);
    //         } else {
    //             fRight.set(0);
    //             mRight.set(0);
    //             bRight.set(0);
    //         }
    //     } else {
    //         fRight.set(0);
    //         mRight.set(0);
    //         bRight.set(0);
    //         if (driveBox.getRawButton(x_button)) {
    //             fLeft.set(.5);
    //             mLeft.set(0);
    //             bLeft.set(0);
    //         } else if (driveBox.getRawButton(y_button)) {
    //             fLeft.set(0);
    //             mLeft.set(.5);
    //             bLeft.set(0);
    //         } else if (driveBox.getRawButton(b_button)) {
    //             fLeft.set(0);
    //             mLeft.set(0);
    //             bLeft.set(.5);
    //         } else {
    //             fLeft.set(0);
    //             mLeft.set(0);
    //             bLeft.set(0);
    //         }
    //     }
    // }

    // public double skim(double v) {
    //     if (v > 1.0)
    //         return -((v - 1.0) * gain_skim);

    //     else if (v < -1.0)
    //         return -((v + 1.0) * gain_skim);

    //     return 0;
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

}