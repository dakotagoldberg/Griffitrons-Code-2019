package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.auto.EchoServer;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Robot extends TimedRobot implements Drive_Constants {
    double x, y, throttle, turn, speedL, speedR;

    WPI_TalonSRX fLeft = new WPI_TalonSRX(0);
    WPI_TalonSRX mLeft = new WPI_TalonSRX(1);
    WPI_TalonSRX bLeft = new WPI_TalonSRX(2);
    WPI_TalonSRX fRight = new WPI_TalonSRX(3);
    WPI_TalonSRX mRight = new WPI_TalonSRX(4);
    WPI_TalonSRX bRight = new WPI_TalonSRX(5);

    EchoServer jetson;

    SpeedControllerGroup left = new SpeedControllerGroup(fLeft, mLeft, bLeft);
    SpeedControllerGroup right = new SpeedControllerGroup(fRight, mRight, bRight);

    DifferentialDrive TestCoast = new DifferentialDrive(left, right);
    Joystick joy = new Joystick(0);

    DoubleSolenoid leftSole = new DoubleSolenoid(0, 1);
    DoubleSolenoid rightSole = new DoubleSolenoid(2, 3);

    @Override
    public void robotInit() {
        jetson = new EchoServer();

        fLeft.configContinuousCurrentLimit(30);
        mLeft.configContinuousCurrentLimit(30);
        bLeft.configContinuousCurrentLimit(30);
        fRight.configContinuousCurrentLimit(30);
        mRight.configContinuousCurrentLimit(30);
        bRight.configContinuousCurrentLimit(30);
        fLeft.configPeakCurrentLimit(30);
        mLeft.configPeakCurrentLimit(30);
        bLeft.configPeakCurrentLimit(30);
        fRight.configPeakCurrentLimit(30);
        mRight.configPeakCurrentLimit(30);
        bRight.configPeakCurrentLimit(30);
        fLeft.configOpenloopRamp(.2);
        mLeft.configOpenloopRamp(.2);
        bLeft.configOpenloopRamp(.2);
        fRight.configOpenloopRamp(.2);
        mRight.configOpenloopRamp(.2);
        bRight.configOpenloopRamp(.2);
    }

    @Override
    public void robotPeriodic() {

    }

    @Override
    public void autonomousInit() {
        jetson.start();
    }

    @Override
    public void autonomousPeriodic() {
        throttle = jetson.getThrottle();
        speedL = throttle;
        speedR = throttle;
        TestCoast.tankDrive(speedL, speedR);
    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {
        y = -joy.getY();
        x = joy.getX();

        if (Math.abs(y) > 0.01)
            throttle = y;
        else
            throttle = 0.0;

        if (Math.abs(x) > 0.25)
            turn = x;
        else
            turn = 0.0;

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

        if (joy.getRawButton(4)) {
            leftSole.set(DoubleSolenoid.Value.kReverse);
            rightSole.set(DoubleSolenoid.Value.kReverse);
        } else if (joy.getRawButton(5)) {
            leftSole.set(DoubleSolenoid.Value.kForward);
            rightSole.set(DoubleSolenoid.Value.kForward);
        } else {
            leftSole.set(DoubleSolenoid.Value.kOff);
            rightSole.set(DoubleSolenoid.Value.kOff);
        }

    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {
        if (joy.getRawButton(1)) {
            fLeft.set(0);
            mLeft.set(0);
            mLeft.set(0);
            if (joy.getRawButton(3)) {
                fRight.set(.5);
                mRight.set(0);
                bRight.set(0);
            } else if (joy.getRawButton(4)) {
                fRight.set(0);
                mRight.set(.5);
                bRight.set(0);
            } else if (joy.getRawButton(5)) {
                fRight.set(0);
                mRight.set(0);
                bRight.set(.5);
            } else {
                fRight.set(0);
                mRight.set(0);
                bRight.set(0);
            }
        } else {
            fRight.set(0);
            mRight.set(0);
            bRight.set(0);
            if (joy.getRawButton(3)) {
                fLeft.set(.5);
                mLeft.set(0);
                bLeft.set(0);
            } else if (joy.getRawButton(4)) {
                fLeft.set(0);
                mLeft.set(.5);
                bLeft.set(0);
            } else if (joy.getRawButton(5)) {
                fLeft.set(0);
                mLeft.set(0);
                bLeft.set(.5);
            } else {
                fLeft.set(0);
                mLeft.set(0);
                bLeft.set(0);
            }
        }
    }

    public double skim(double v) {
        if (v > 1.0)
            return -((v - 1.0) * gain_skim);

        else if (v < -1.0)
            return -((v + 1.0) * gain_skim);

        return 0;
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

}