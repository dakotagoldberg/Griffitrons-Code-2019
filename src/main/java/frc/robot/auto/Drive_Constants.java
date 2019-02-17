package frc.robot.auto;

public interface Drive_Constants {
    final static double gain_skim = 0.9;
    final static double gain_turn = 1.0;

    final static double feet_to_meters = 0.3048;

    final static double time_step = 0.05; // s

    final static double max_vel = 5.64; // m/s

    final static double torque_per_motor = 2.425; // N*m
    final static int num_motors = 6;
    final static double low_gear_ratio = 0.115;
    final static double high_gear_ratio = 0.277;

    final static double torque = torque_per_motor * num_motors * high_gear_ratio; // N*m

    final static double wheel_radius = .0508; // m
    final static double robot_weight = 56.7; // kg
    final static double max_accel = torque / (wheel_radius * robot_weight); // m/s/s

    final static double max_jerk = 60.0; // m/s/s/s

    final static double wheelbase_width = 28.0 / 12 * feet_to_meters;

    final static int continuous_current = 30;
    final static int peak_current = 30;
    final static double open_ramp = 0.2;
    
}