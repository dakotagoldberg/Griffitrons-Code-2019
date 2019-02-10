
package frc.robot.auto;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import jaci.pathfinder.*;
import jaci.pathfinder.Trajectory.FitMethod;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier; 

/**
 * Add your docs here.
 */
public class Drive {
    //this is not the actual wheel base width, when it is the actual wheel base width please delete me!
    //this value is in meters vvv
    double wheelbase_width = 0.5;
    private WPI_TalonSRX L1 = new WPI_TalonSRX(0);
    private WPI_TalonSRX L2 = new WPI_TalonSRX(1);
    private WPI_TalonSRX L3 = new WPI_TalonSRX(2);
    private WPI_TalonSRX R1 = new WPI_TalonSRX(3);
    private WPI_TalonSRX R2 = new WPI_TalonSRX(4);
    private WPI_TalonSRX R3 = new WPI_TalonSRX(5);

    SpeedControllerGroup leftSpd = new SpeedControllerGroup(L1, L2, L3);
    SpeedControllerGroup rightSpd = new SpeedControllerGroup(R1, R2, R3);

    Trajectory.Config config = new Trajectory.Config(FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH,0.05,1.7,2.0,60.0);
    double feetToMeters = 0.3048;

    public void autoPath(int path){
        //int fed from dash board determines path then it follows apropriate tape
    }
    public void setpoint(int x, int y, int theta){
        x *= feetToMeters;
        y *= feetToMeters;
        //give it a point and it goes there, theta for rotation
        //creates a single point as a single variable array because why not
        Waypoint[] point = {new Waypoint(x,y,theta)};
        
        Trajectory trajectory = Pathfinder.generate(point, config);
        //takes the newly calculated trajectory and modifies the values iwth the handy dandy modify methods
        TankModifier modifier = new TankModifier(trajectory);
        modifier.modify(wheelbase_width);
        Trajectory leftTraj = modifier.getLeftTrajectory();
        Trajectory rightTraj = modifier.getRightTrajectory();
        
        EncoderFollower left = new EncoderFollower(leftTraj);
        EncoderFollower right = new EncoderFollower(rightTraj);

        left.configureEncoder(getEncPosition(), 1000, wheel_diameter);
        right.configureEncoder(getEncoderPosition(), 1000, wheel_diameter);

        left.configurePIDVA(kp, ki, kd, kv, ka);
        right.configurePIDVA(kp, ki, kd, kv, ka);

        double lOutput = left.calculate(getEncoderPosition());
        double rOutput = left.calculate(getEncoderPosition());

        leftSpd.set(lOutput);
        rightSpd.set(rOutput);
    }
    public void setpointArray(int[] cords){
        //set point but it recieves array from GetGoal;
    }

}
