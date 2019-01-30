/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Add your docs here.
 */
public class PneumaticsTest extends TimedRobot {
    DoubleSolenoid sole = new DoubleSolenoid(3, 2);
    Joystick joy = new Joystick(0);
	WPI_TalonSRX bLeft = new WPI_TalonSRX(1);

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {
        double y = -joy.getRawAxis(1);
        if(Math.abs(y) > 0.05)
            bLeft.set(y);
        else 
            bLeft.set(0);

        if(joy.getRawButton(3) && bLeft.get() > 0.05)
            sole.set(DoubleSolenoid.Value.kReverse);
        else if(joy.getRawButton(4) && bLeft.get() > 0.05)
            sole.set(DoubleSolenoid.Value.kForward);
        else
            sole.set(DoubleSolenoid.Value.kOff);
    }

}
