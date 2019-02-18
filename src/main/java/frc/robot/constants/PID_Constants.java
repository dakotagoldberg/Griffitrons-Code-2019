package frc.robot.constants;

public interface PID_Constants {
    double degreesToTicks = 4096./360;
    int kTimeoutMs = 10;

    //Claw Control
    double left_claw_closed = 53; 
    double left_claw_open = 319;

    double right_claw_closed = 0;
    double right_claw_open = 0;

    double claw_f = 1.17;

    int claw_cruise_velocity = 737;
    int claw_cruise_accel = 737;

    //Intake Rotate Control

    //Elevato Control
}