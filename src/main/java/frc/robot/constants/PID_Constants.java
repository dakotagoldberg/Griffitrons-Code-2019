package frc.robot.constants;

public interface PID_Constants {
    double degreesToTicks = 4096./360;
    int kTimeoutMs = 10;
    int ticks_per_revolution = 4096;

    //Drive Control

    double drive_p = 0.0;
    double drive_i = 0.0;
    double drive_d = 0.0;

    //Claw Control
    double left_claw_open = -219;
    double left_claw_ball = 82;
    double left_claw_closed = 314; 

    double right_claw_open = 4456;
    double right_claw_ball = 4161;
    double right_claw_closed = 3775;


    double claw_p = 4.0;
    double claw_i = 0.008;
    double claw_d = 180.0;
    double claw_f = 1.17;

    int claw_cruise_velocity = 737;
    int claw_cruise_accel = 737;

    //Intake Rotate Control
    double intake_rotate_hatch = 0; 
    double intake_rotate_ball = 0;

    double intake_rotate_p = 4.0;
    double intake_rotate_i = 0.008;
    double intake_rotate_d = 180.0;
    double intake_rotate_f = 1.17;

    int intake_rotate_cruise_velocity = 737;
    int intake_rotate_cruise_accel = 737;

    //Elevator Control
    double left_elev_hatch_top = 314; 
    double left_elev_hatch_middle = -219;
    double left_elev_hatch_bottom = 82;

    double left_elev_ball_top = 3775;
    double left_elev_ball_middle = 4456;
    double left_elev_ball_bottom = 4161;

    double right_elev_hatch_top = 314; 
    double right_elev_hatch_middle = -219;
    double right_elev_hatch_bottom = 82;

    double right_elev_ball_top = 3775;
    double right_elev_ball_middle = 4456;
    double right_elev_ball_bottom = 4161;

    double elev_p = 4.0;
    double elev_i = 0.008;
    double elev_d = 180.0;
    double elev_f = 1.17;

    int elev_cruise_velocity = 737;
    int elev_cruise_accel = 737;
}