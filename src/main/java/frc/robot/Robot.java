package frc.robot;

import frc.robot.auto.AutoDrive;
import frc.robot.constants.Robot_Framework;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Robot extends TimedRobot implements Robot_Framework {
    AutoDrive auto;

    @Override
    public void robotInit() {
        
    }

    @Override
    public void robotPeriodic() {
        dash.update();
    }

    @Override
    public void autonomousInit() {
        jetson.start();
        auto = new AutoDrive(dash.getAutoMode(), dash.getPants());
    }

    @Override
    public void autonomousPeriodic() {
        auto.execute();
    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {
        drive.executeTank(); //Uses both sticks on driveBox        

        if (driveBox.getRawButton(left_bumper)) {
            gearSole.set(DoubleSolenoid.Value.kReverse);
            gearSole.set(DoubleSolenoid.Value.kReverse);
        } else if (driveBox.getRawButton(right_bumper)) {
            gearSole.set(DoubleSolenoid.Value.kForward);
            gearSole.set(DoubleSolenoid.Value.kForward);
        } else {
            gearSole.set(DoubleSolenoid.Value.kOff);
            gearSole.set(DoubleSolenoid.Value.kOff);
        }

        if (driveBox.getTriggerAxis(Hand.kLeft) >= 0.02){
            leftIntake.set(-driveBox.getTriggerAxis(Hand.kLeft));
            rightIntake.set(-driveBox.getTriggerAxis(Hand.kLeft));
        } else if (driveBox.getTriggerAxis(Hand.kRight) >= 0.02){
            leftIntake.set(driveBox.getTriggerAxis(Hand.kRight));
            rightIntake.set(driveBox.getTriggerAxis(Hand.kRight));
        } else {
            leftIntake.set(0);
            rightIntake.set(0);
        }

        
    }

    @Override
    public void testInit() {
        
    }

    @Override
    public void testPeriodic() {
        if(driveBox.getRawButton(left_bumper))
            claws.hatchRelease();
        else if(driveBox.getRawButton(right_bumper))
            claws.hatchGrab();
        else if(driveBox.getRawButton(b_button))
            claws.ballGrab();
        System.out.println(leftElev.getSensorCollection().getPulseWidthPosition() + " "
                        + rightClaw.getSensorCollection().getPulseWidthPosition());
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

}