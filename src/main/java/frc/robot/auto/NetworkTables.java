package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.*;

public class NetworkTables {

    public NetworkTables() {
        SmartDashboard.putNumber("p", 0.012);
		SmartDashboard.putNumber("i", 0.00);
        SmartDashboard.putNumber("d", 0.00);
        
        SmartDashboard.putNumber("gyro", 0.00);
        SmartDashboard.putBoolean("gyroReset", false);
        
        SmartDashboard.putNumber("flEnc", 0);
		SmartDashboard.putNumber("blEnc", 0);
		SmartDashboard.putNumber("frEnc", 0);
		SmartDashboard.putNumber("brEnc", 0);
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
		SmartDashboard.putBoolean("pants", false);
    }

    public boolean getBoolean(String key) {
        return SmartDashboard.getBoolean(key, false);
    }

    public double getNumber(String key) {
        return SmartDashboard.getNumber(key, 0);
    }

    public void putBoolean(String key, boolean value) {
        SmartDashboard.putBoolean(key, value);
    }

    public void putNumber(String key, double value) {
        SmartDashboard.putNumber(key, value);
    }
}