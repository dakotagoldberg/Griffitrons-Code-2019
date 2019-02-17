package frc.robot.auto;

import frc.robot.constants.Robot_Framework;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.hal.PowerJNI;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class NetworkTables implements Robot_Framework {

    public NetworkTables() {
        SmartDashboard.putNumber("p", 0.012);
		SmartDashboard.putNumber("i", 0.00);
        SmartDashboard.putNumber("d", 0.00);
        
        SmartDashboard.putNumber("gyro", 0.00);
        SmartDashboard.putBoolean("gyroReset", false);
        
        SmartDashboard.putNumber("lEnc", 0);
		SmartDashboard.putNumber("rEnc", 0);
		SmartDashboard.putNumber("intakeEnc", 0);
        SmartDashboard.putNumber("rotationEnc", 0);
        SmartDashboard.putNumber("elevatorEnc", 0);
        SmartDashboard.putBoolean("encReset", false);
        
        SmartDashboard.putNumber("flDrive", 0);
		SmartDashboard.putNumber("frDrive", 0);
		SmartDashboard.putNumber("blDrive", 0);
		SmartDashboard.putNumber("brDrive", 0);
		SmartDashboard.putNumber("intake", 0);
		SmartDashboard.putNumber("intakerotate", 0);
		SmartDashboard.putNumber("elevator", 0);
        SmartDashboard.putNumber("climber", 0);
        
        SmartDashboard.putNumber("timer", 135);
        SmartDashboard.putBoolean("inauto", false);
        
        SmartDashboard.putNumber("voltage", 0);
		SmartDashboard.putNumber("totaldraw", 0);
		SmartDashboard.putNumber("drivedraw", 0);
		SmartDashboard.putNumber("intakedraw", 0);
		SmartDashboard.putNumber("intakerotatedraw", 0);
		SmartDashboard.putNumber("elevatordraw", 0);
        SmartDashboard.putNumber("climberdraw", 0);
        
        SmartDashboard.putNumber("velocity", 0);
		SmartDashboard.putNumber("acceleration", 0);
        SmartDashboard.putNumber("temperature", 0);
        
        SmartDashboard.putBoolean("isred", false);

        SmartDashboard.putNumber("automode", 0);
    }

    public void update() {
        /**
		 * In this section, the variables that were initialized in robotInit are continuously updated and read from.
		 * This if statement checks if the gyroReset button was clicked on the dashboard, and reset the gyro if so.
		 */
		if(SmartDashboard.getBoolean("gyroReset", false)) {
			gyro.reset();
			SmartDashboard.putBoolean("gyroReset", false);
		}

		/**
		 * This sends the current gyro angle to the dashboard.
		 */
		SmartDashboard.putNumber("gyro", gyro.getAngle());

		/**
		 * This checks if the encoder reset button was pressed, and reset encoders if so.
		 */
		if(SmartDashboard.getBoolean("encReset", false)) {
			leftDriveEnc.reset();
			rightDriveEnc.reset();
			iRotateEnc.reset();
            elevEnc.reset();
            leftClawEnc.reset();
            rightClawEnc.reset();
            climbEnc.reset();
			SmartDashboard.putBoolean("encReset", false);
		}

		/**
		 * This sends the current distance (converted from inches to feet) that the encoders have turned.
		 */
		SmartDashboard.putNumber("lEnc", leftDriveEnc.getDistance() / 12);
		SmartDashboard.putNumber("rEnc", rightDriveEnc.getDistance() / 12);
		SmartDashboard.putNumber("rotationEnc", iRotateEnc.getDistance() / 12);
        SmartDashboard.putNumber("elevatorEnc", elevEnc.getDistance() / 12);
        SmartDashboard.putNumber("intakeEnc", leftClawEnc.getDistance() / 12);
		SmartDashboard.putNumber("intakeEnc2", rightClawEnc.getDistance() / 12);
        SmartDashboard.putNumber("climbEnc", climbEnc.getDistance() / 12);

        /**
		 * Motor values are sent to the dashboard only one of each type is sent for the simplicity
		 * of the dashboard.
		 */
        SmartDashboard.putNumber("flDrive", fLeft.get());
        SmartDashboard.putNumber("mlDrive", mLeft.get());
        SmartDashboard.putNumber("blDrive", bLeft.get());
        SmartDashboard.putNumber("frDrive", fRight.get());
        SmartDashboard.putNumber("mrDrive", mRight.get());
        SmartDashboard.putNumber("brDrive", bRight.get());

		SmartDashboard.putNumber("lClaw", leftClaw.get());
		SmartDashboard.putNumber("rClaw", rightClaw.get());
		SmartDashboard.putNumber("lElev", leftElev.get());
        SmartDashboard.putNumber("rElev", rightElev.get());
        SmartDashboard.putNumber("iRotate", iRotate.get());
        SmartDashboard.putNumber("lIntake", leftIntake.get());
        SmartDashboard.putNumber("rIntake", rightIntake.get());
        SmartDashboard.putNumber("lClimb", leftClimb.get());
        SmartDashboard.putNumber("rClimb", rightClimb.get());

		/**
		 * The current match time and auto status are sent to the dashboard for the countdown timer.
		 */
		SmartDashboard.putNumber("timer", ds.getMatchTime());
		SmartDashboard.putBoolean("inauto", ds.isAutonomous());

		/**
		 * The current voltage and current of each motor is sent to the dashboard continuously.
		 */
		SmartDashboard.putNumber("voltage", RobotController.getBatteryVoltage());
		SmartDashboard.putNumber("totaldraw", PowerJNI.getVinCurrent());
        SmartDashboard.putNumber("drivedraw", fLeft.getOutputCurrent() + mLeft.getOutputCurrent() 
                                            + bLeft.getOutputCurrent() + fRight.getOutputCurrent() 
                                            + mRight.getOutputCurrent() + bRight.getOutputCurrent());
        SmartDashboard.putNumber("clawdraw", leftClaw.getOutputCurrent() + rightClaw.getOutputCurrent());
        SmartDashboard.putNumber("intakedraw", leftIntake.getOutputCurrent() + rightIntake.getOutputCurrent());
		SmartDashboard.putNumber("intakerotatedraw", iRotate.getOutputCurrent());
		SmartDashboard.putNumber("elevatordraw", leftElev.getOutputCurrent() + rightElev.getOutputCurrent());
		SmartDashboard.putNumber("climberdraw", leftClimb.getOutputCurrent() + rightClimb.getOutputCurrent());

		/**
		 * These bonus data pieces are also continuously updated on the smart dashboard.
		 */
		SmartDashboard.putNumber("velocity", Math.sqrt(gyro.getVelocityX() * gyro.getVelocityX() + gyro.getVelocityY() * gyro.getVelocityY()));
		SmartDashboard.putNumber("acceleration", Math.sqrt(gyro.getWorldLinearAccelX() * gyro.getWorldLinearAccelX()  + gyro.getWorldLinearAccelY() * gyro.getWorldLinearAccelY()));
		SmartDashboard.putNumber("temperature", gyro.getTempC());

		/**
		 * While not strictly necessary, our current alliance (red or blue) is continuously sent to the dashboard.
		 */
		SmartDashboard.putBoolean("isred", ds.getAlliance().equals(Alliance.Red));
    }

    public double getP() {
        return SmartDashboard.getNumber("p", 0);
    }

    public double getI() {
        return SmartDashboard.getNumber("i", 0);
    }

    public double getD() {
        return SmartDashboard.getNumber("d", 0);
    }

    public int getAutoMode() {
        return (int)SmartDashboard.getNumber("automode", 0);
    }

}