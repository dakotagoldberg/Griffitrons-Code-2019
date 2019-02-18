package frc.robot.auto;

import frc.robot.constants.Robot_Framework;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.hal.PowerJNI;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class NetworkTables implements Robot_Framework {

    public NetworkTables() {
        SmartDashboard.putNumber("p", 0.00); //starting PID values
		SmartDashboard.putNumber("i", 0.00); 
        SmartDashboard.putNumber("d", 0.00);
        
        SmartDashboard.putNumber("gyro", 0.00);
        SmartDashboard.putBoolean("gyroReset", false);
        
		SmartDashboard.putNumber("lEnc", 0);
		SmartDashboard.putBoolean("lEncReset", false);
		SmartDashboard.putNumber("rEnc", 0);
		SmartDashboard.putBoolean("rEncReset", false);
		SmartDashboard.putNumber("intakeEnc", 0);
		SmartDashboard.putBoolean("intakeEncReset", false);
		SmartDashboard.putNumber("rotationEnc", 0);
		SmartDashboard.putBoolean("rotationEncReset", false);
        SmartDashboard.putNumber("elevatorEnc", 0);
        SmartDashboard.putBoolean("elevatorEncReset", false);
        
        SmartDashboard.putNumber("lDrive", 0);
		SmartDashboard.putNumber("rDrive", 0);
		SmartDashboard.putNumber("leftClawIntake", 0);
		SmartDashboard.putNumber("rightClawIntake", 0);
		SmartDashboard.putNumber("leftBallIntake", 0);
		SmartDashboard.putNumber("rightBallIntake", 0);
		SmartDashboard.putNumber("intakerotate", 0);
		SmartDashboard.putNumber("elevator", 0);
        // SmartDashboard.putNumber("climber", 0);
        
        SmartDashboard.putNumber("timer", 135);
        SmartDashboard.putBoolean("inauto", false);
        
        SmartDashboard.putNumber("voltage", 0);
		SmartDashboard.putNumber("totaldraw", 0);
		SmartDashboard.putNumber("drivedraw", 0);
		SmartDashboard.putNumber("intakedraw", 0);
		SmartDashboard.putNumber("intakerotatedraw", 0);
		SmartDashboard.putNumber("elevatordraw", 0);
        // SmartDashboard.putNumber("climberdraw", 0);
        
        SmartDashboard.putNumber("velocity", 0);
		SmartDashboard.putNumber("acceleration", 0);
        SmartDashboard.putNumber("temperature", 0);
        
        SmartDashboard.putBoolean("isred", false);

		SmartDashboard.putNumber("automode", 0);
		
		SmartDashboard.putBoolean("jetsonConnected", false);
		SmartDashboard.putString("consoleOutput", "");
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
		if(SmartDashboard.getBoolean("lEncReset", false)) {
			fLeft.getSensorCollection().setPulseWidthPosition(0, kTimeoutMs);
			SmartDashboard.putBoolean("lEncReset", false);
		}
		if(SmartDashboard.getBoolean("rEncReset", false)) {
			fRight.getSensorCollection().setPulseWidthPosition(0, kTimeoutMs);
			SmartDashboard.putBoolean("rEncReset", false);
		}
		if(SmartDashboard.getBoolean("intakeEncReset", false)) {
			leftClaw.getSensorCollection().setPulseWidthPosition(0, kTimeoutMs);
			SmartDashboard.putBoolean("intakeEncReset", false);
		}
		if(SmartDashboard.getBoolean("rotationEncReset", false)) {
			iRotate.getSensorCollection().setPulseWidthPosition(0, kTimeoutMs);
			SmartDashboard.putBoolean("rotationEncReset", false);
		}
		if(SmartDashboard.getBoolean("elevatorEncReset", false)) {
			leftElev.getSensorCollection().setPulseWidthPosition(0, kTimeoutMs);
			SmartDashboard.putBoolean("elevatorEncReset", false);
		}

		/**
		 * This sends the current distance (converted from inches to feet) that the encoders have turned.
		 */
		// SmartDashboard.putNumber("lEnc", fLeft.getSensorCollection().getPulseWidthPosition());
		// SmartDashboard.putNumber("rEnc", fRight.getSensorCollection().getPulseWidthPosition());
		SmartDashboard.putNumber("intakeEnc", leftClaw.getSensorCollection().getPulseWidthPosition());
        SmartDashboard.putNumber("rotationEnc", rightClaw.getSensorCollection().getPulseWidthPosition());
        // SmartDashboard.putNumber("elevatorEnc", leftElev.getSensorCollection().getPulseWidthPosition());

        /**
		 * Motor values are sent to the dashboard only one of each type is sent for the simplicity
		 * of the dashboard.
		 */
        SmartDashboard.putNumber("lDrive", fLeft.get());
		SmartDashboard.putNumber("rDrive", fRight.get());
		SmartDashboard.putNumber("leftClawIntake", leftClaw.get());
		SmartDashboard.putNumber("rightClawIntake", rightClaw.get());
		SmartDashboard.putNumber("leftBallIntake", leftIntake.get());
		SmartDashboard.putNumber("rightBallIntake", rightIntake.get());
		SmartDashboard.putNumber("intakerotate", iRotate.get());
		SmartDashboard.putNumber("elevator", leftElev.get());

		/**
		 * The current match time and auto status are sent to the dashboard for the countdown timer.
		 */
		SmartDashboard.putNumber("timer", ds.getMatchTime());
		SmartDashboard.putBoolean("inauto", ds.isAutonomous());

		/**
		 * The current voltage and current of each motor is sent to the dashboard continuously.
		 */
		// SmartDashboard.putNumber("voltage", RobotController.getBatteryVoltage());
		// SmartDashboard.putNumber("totaldraw", PowerJNI.getVinCurrent());
        // SmartDashboard.putNumber("drivedraw", fLeft.getOutputCurrent() + mLeft.getOutputCurrent() 
        //                                     + bLeft.getOutputCurrent() + fRight.getOutputCurrent() 
        //                                     + mRight.getOutputCurrent() + bRight.getOutputCurrent());
		// SmartDashboard.putNumber("clawdraw", leftClaw.getOutputCurrent() + rightClaw.getOutputCurrent()
		// 									+ leftIntake.getOutputCurrent() + rightIntake.getOutputCurrent());
		// SmartDashboard.putNumber("intakerotatedraw", iRotate.getOutputCurrent());
		// SmartDashboard.putNumber("elevatordraw", leftElev.getOutputCurrent() + rightElev.getOutputCurrent());
		// SmartDashboard.putNumber("climberdraw", leftClimb.getOutputCurrent() + rightClimb.getOutputCurrent());

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