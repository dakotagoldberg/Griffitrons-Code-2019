package frc.robot.teleop;

import frc.robot.constants.Robot_Framework;

public class Drive implements Robot_Framework {

    double x, y, throttle, turn, speedL, speedR, t_left, t_right;

    public Drive() { 
        fLeft.configContinuousCurrentLimit(continuous_current);
        mLeft.configContinuousCurrentLimit(continuous_current);
        bLeft.configContinuousCurrentLimit(continuous_current);
        fRight.configContinuousCurrentLimit(continuous_current);
        mRight.configContinuousCurrentLimit(continuous_current);
        bRight.configContinuousCurrentLimit(continuous_current);
        fLeft.configPeakCurrentLimit(peak_current);
        mLeft.configPeakCurrentLimit(peak_current);
        bLeft.configPeakCurrentLimit(peak_current);
        fRight.configPeakCurrentLimit(peak_current);
        mRight.configPeakCurrentLimit(peak_current);
        bRight.configPeakCurrentLimit(peak_current);
        fLeft.configOpenloopRamp(open_ramp);
        mLeft.configOpenloopRamp(open_ramp);
        bLeft.configOpenloopRamp(open_ramp);
        fRight.configOpenloopRamp(open_ramp);
        mRight.configOpenloopRamp(open_ramp);
        bRight.configOpenloopRamp(open_ramp);
    }

    public void executeTank() {
        y = -driveBox.getRawAxis(left_y_axis);
        x = driveBox.getRawAxis(right_x_axis);

        if (Math.abs(y) > 0.1)
            throttle = y;
        else
            throttle = 0.0;

        if (Math.abs(x) > 0.1)
            turn = x;
        else
            turn = 0.0;

        t_left = throttle + turn;
        t_right = throttle - turn;

        speedL = t_left + skim(t_right);
        speedR = t_right + skim(t_left);

        tank.tankDrive(speedL, speedR);
    }

    private double skim(double v) {
        if (v > 1.0)
            return -((v - 1.0) * gain_skim);        
        else if (v < -1.0)
            return -((v + 1.0) * gain_skim);
        return 0;
    }
}