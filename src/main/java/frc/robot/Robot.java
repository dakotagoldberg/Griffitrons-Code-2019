package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Robot extends TimedRobot implements Drive_Constants {
    double x, y, throttle, turn, speedL, speedR;

    WPI_TalonSRX fLeft = new WPI_TalonSRX(0);
    WPI_TalonSRX mLeft = new WPI_TalonSRX(1);
    WPI_TalonSRX bLeft = new WPI_TalonSRX(2);
    WPI_TalonSRX fRight = new WPI_TalonSRX(3);
    WPI_TalonSRX mRight = new WPI_TalonSRX(4);
    WPI_TalonSRX bRight = new WPI_TalonSRX(5);

    SpeedControllerGroup left = new SpeedControllerGroup(fLeft, mLeft, bLeft);
    SpeedControllerGroup right = new SpeedControllerGroup(fRight, mRight, fLeft);

    DifferentialDrive TestCoast = new DifferentialDrive(left, right);
    Joystick joy = new Joystick(0);

    DoubleSolenoid sole = new DoubleSolenoid(0, 1);

    @Override
    public void robotInit() {
        fRight.setInverted(true);
        mRight.setInverted(true);
        bRight.setInverted(true);
    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {
        y = -joy.getY();
        x = joy.getX();

        if (Math.abs(y) > 0.5)
            throttle = y;
        else
            throttle = 0.0;

        if (Math.abs(x) > 0.5)
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

        speedL = t_left;// + skim(t_right);
        speedR = t_right;// + skim(t_left);

        TestCoast.tankDrive(speedL, speedR);

        if (joy.getRawButton(4) && Math.abs(bLeft.get()) > 0.05)
            sole.set(DoubleSolenoid.Value.kReverse);
        else if (joy.getRawButton(5) && Math.abs(bLeft.get()) > 0.05) {
            sole.set(DoubleSolenoid.Value.kForward);
        } else
            sole.set(DoubleSolenoid.Value.kOff);

    }

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

}