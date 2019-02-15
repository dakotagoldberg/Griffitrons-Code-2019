
package frc.robot.auto;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.*;

import jaci.pathfinder.*;
import jaci.pathfinder.Trajectory.FitMethod;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier; 

public class Drive implements Drive_Constants {

    static Trajectory.Config config = new Trajectory.Config(FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 
        time_step, max_vel, max_accel, max_jerk);

    static Waypoint[] points;

    public static void autoPath(int path){
        if(path == 0) { // 0 = LEFT START
            points = new Waypoint[] {
                new Waypoint(1, 1, Pathfinder.d2r(45)),
                new Waypoint(-1, 0, 0)
            };
        } else if(path == 1) { // 1 = MIDDLE START
            points = new Waypoint[] {
                new Waypoint(0, 1, 0),
                new Waypoint(0, 0, Pathfinder.d2r(120)),
                new Waypoint(0, 1, Pathfinder.d2r(120)),
                new Waypoint(0, 0, Pathfinder.d2r(120)),
                new Waypoint(0, 1, 0)
            };
        } else { // 2 = RIGHT START
            points = new Waypoint[] {
                new Waypoint(0.5, 1, 0),
                new Waypoint(-1, 0, 0),
                new Waypoint(0, -1, 0)
            };
        }
    }

    public static void execute(SpeedControllerGroup left, SpeedControllerGroup right){
        
        //Choose path
        autoPath(1);
        
        //Calculate trajectory
        Trajectory trajectory = Pathfinder.generate(points, config);
        
        //Modify trajectory for tank
        TankModifier modifier = new TankModifier(trajectory);
        modifier.modify(wheelbase_width);

        Trajectory leftTraj = modifier.getLeftTrajectory();
        Trajectory rightTraj = modifier.getRightTrajectory();
        
        //Feed trajectories to encoders
        EncoderFollower leftEnc = new EncoderFollower(leftTraj);
        EncoderFollower rightEnc = new EncoderFollower(rightTraj);

        leftEnc.configureEncoder(0, 1000, 2 * wheel_radius);
        rightEnc.configureEncoder(0, 1000, 2 * wheel_radius);
 
        //Tune PID
        leftEnc.configurePIDVA(kp, ki, kd, 0, 0);
        rightEnc.configurePIDVA(kp, ki, kd, 0, 0);

        while(!(leftEnc.isFinished() && rightEnc.isFinished())) {
            double lOutput = leftEnc.calculate(encoder_tick);
            double rOutput = leftEnc.calculate(encoder_tick);
            left.set(lOutput);
            right.set(rOutput);
        }
    }

}
