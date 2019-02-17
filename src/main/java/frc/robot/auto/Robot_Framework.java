package frc.robot.auto;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.DigitalInput;


public interface Robot_Framework extends Drive_Constants, Control_Constants, ID_Constants {
    EchoServer jetson = new EchoServer();
    NetworkTables dash = new NetworkTables();
    Intake claws = new Intake();
    DriverStation ds = DriverStation.getInstance();
    
    WPI_TalonSRX fLeft = new WPI_TalonSRX(front_left_drive);
    WPI_TalonSRX mLeft = new WPI_TalonSRX(middle_left_drive);
    WPI_TalonSRX bLeft = new WPI_TalonSRX(back_left_drive);
    WPI_TalonSRX fRight = new WPI_TalonSRX(front_right_drive);
    WPI_TalonSRX mRight = new WPI_TalonSRX(middle_right_drive);
    WPI_TalonSRX bRight = new WPI_TalonSRX(back_right_drive);

    WPI_TalonSRX leftClaw = new WPI_TalonSRX(left_claw);
    WPI_TalonSRX rightClaw = new WPI_TalonSRX(right_claw);
    WPI_TalonSRX leftElev = new WPI_TalonSRX(left_elevator);
    WPI_TalonSRX rightElev = new WPI_TalonSRX(right_elevator);
    WPI_TalonSRX iRotate = new WPI_TalonSRX(intake_rotation);
    WPI_TalonSRX leftIntake = new WPI_TalonSRX(left_intake);
    WPI_TalonSRX rightIntake = new WPI_TalonSRX(right_intake);
    WPI_TalonSRX leftClimb = new WPI_TalonSRX(left_climb);
    WPI_TalonSRX rightClimb = new WPI_TalonSRX(right_climb);

    Encoder leftDriveEnc = new Encoder(0, 1, false, CounterBase.EncodingType.k4X);
    Encoder rightDriveEnc = new Encoder(2, 3, false, CounterBase.EncodingType.k4X);
    Encoder iRotateEnc = new Encoder(4, 5, false, CounterBase.EncodingType.k4X);
    Encoder elevEnc = new Encoder(6, 7, false, CounterBase.EncodingType.k4X);
    Encoder leftClawEnc = new Encoder(8, 9, false, CounterBase.EncodingType.k4X);
    Encoder rightClawEnc = new Encoder(10, 11, false, CounterBase.EncodingType.k4X);
    Encoder climbEnc = new Encoder(12, 13, false, CounterBase.EncodingType.k4X); 

    DigitalInput bottomElevLim = new DigitalInput(bottom_elevator_limit);
    DigitalInput topElevLim = new DigitalInput(top_elevator_limit);
    DigitalInput iRotateLim = new DigitalInput(intake_rotate_limit);

    SpeedControllerGroup left = new SpeedControllerGroup(fLeft, mLeft, bLeft);
    SpeedControllerGroup right = new SpeedControllerGroup(fRight, mRight, bRight);

    DifferentialDrive drive = new DifferentialDrive(left, right);

    XboxController driveBox = new XboxController(drive_controller);
    XboxController intakeBox = new XboxController(intake_controller);

    DoubleSolenoid gearSole = new DoubleSolenoid(0, 1);
    DoubleSolenoid climbSole1 = new DoubleSolenoid(2, 3);
    DoubleSolenoid climbSole2 = new DoubleSolenoid(4, 5);
    DoubleSolenoid wingSole1 = new DoubleSolenoid(6, 7);
    DoubleSolenoid wingSole2 = new DoubleSolenoid(8, 9);

    AHRS gyro = new AHRS(I2C.Port.kMXP);

}