package frc.robot.auto;

import frc.robot.constants.Robot_Framework;

// import edu.wpi.first.wpilibj.Notifier;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class AutoDrive implements Robot_Framework {

    Trajectory leftPath;
    Trajectory rightPath;

    EncoderFollower leftFollow;
    EncoderFollower rightFollow;

    public AutoDrive(int path, boolean pants) {
        if(path == 0) {
            leftPath = PathfinderFRC.getTrajectory("left.left");
            rightPath = PathfinderFRC.getTrajectory("left.right");
        } else if(path == 1) {
            if(pants) {
                leftPath = PathfinderFRC.getTrajectory("middle_left.left");
                rightPath = PathfinderFRC.getTrajectory("middle_left.right");
            } else {
                leftPath = PathfinderFRC.getTrajectory("middle_right.left");
                rightPath = PathfinderFRC.getTrajectory("middle_right.right");
            }
        } else {
            leftPath = PathfinderFRC.getTrajectory("right.left");
            rightPath = PathfinderFRC.getTrajectory("right.right");
        }

        leftFollow = new EncoderFollower(leftPath);
        rightFollow = new EncoderFollower(rightPath);

        leftFollow.configureEncoder(fLeft.getSensorCollection().getPulseWidthPosition(), 
            ticks_per_revolution, 2 * wheel_radius);
        rightFollow.configureEncoder(fRight.getSensorCollection().getPulseWidthPosition(), 
            ticks_per_revolution, 2 * wheel_radius);
        
        leftFollow.configurePIDVA(drive_p, drive_i, drive_d, 1 / max_vel, 0);
        rightFollow.configurePIDVA(drive_p, drive_i, drive_d, 1 / max_vel, 0);
    }

    public void execute(){
        if(!(leftFollow.isFinished() && rightFollow.isFinished())) {
            double left_speed = leftFollow.calculate(fLeft.getSensorCollection().getPulseWidthPosition());
            double right_speed = rightFollow.calculate(fRight.getSensorCollection().getPulseWidthPosition());
            double heading = gyro.getAngle();
            double desired_heading = Pathfinder.r2d(leftFollow.getHeading());
            double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
            double turn =  0.8 * (-1.0/80.0) * heading_difference;
            left.set(left_speed + turn);
            right.set(right_speed - turn);
        }
    }

}
