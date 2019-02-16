/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

/**
 * Add your docs here.
 */
public class Intake {

    private WPI_TalonSRX c1 = new WPI_TalonSRX(10);
    private WPI_TalonSRX c2 = new WPI_TalonSRX(11);
    // private WPI_TalonSRX w1 = new WPI_TalonSRX(12);
    // private WPI_TalonSRX w2 = new WPI_TalonSRX(13);
    // private WPI_TalonSRX rotation = new WPI_TalonSRX(14);
    private double h1zero = 42, h2zero = 42;
    private double clawClosed = 42, clawOpened = 42;

    private static final int kPIDLoopIdx = 0;
    private static final int kTimeoutMs = 30;
    private static final int kSlotIdx = 0;

    private static final double kp = 0.2, ki = 0, kd = 0, kf = 0.2, izone = 0, peakOutput = 1;

    public void ballIntake() {
        // turns rolllers and such to intake ball
    }

    public void ballShoot() {
        // turns rolllers and such to shoot ball
    }

    public void hatchIntake() {
        // turns rolllers and such to intake hatch
        c1.set(ControlMode.MotionMagic,clawOpened);
        c2.set(ControlMode.MotionMagic,-clawOpened);
    }

    public void hatchShoot() {
        // turns rolllers and such to release hatch
        c1.set(ControlMode.MotionMagic,clawClosed);
        c2.set(ControlMode.MotionMagic,-clawClosed);
    }

    public void zeroRotation() {
        // zeros the Rotation
    }

    public void rotateTO(int degrees) {
        // rotates the input to that degree amount
    }

    public Intake(){
        c2.configFactoryDefault();
        c1.configFactoryDefault();
        
        c1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, kPIDLoopIdx,kTimeoutMs );
        c2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, kPIDLoopIdx,kTimeoutMs );

        c1.setSensorPhase(true);
        c2.setSensorPhase(true);
        c1.setInverted(false);
        c2.setInverted(false);

        c1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,10,kTimeoutMs);
        c2.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,10,kTimeoutMs);
        c1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,10,kTimeoutMs);
        c2.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,10,kTimeoutMs);
        
        c1.configNominalOutputForward(0,kTimeoutMs);
        c2.configNominalOutputForward(0,kTimeoutMs);

        c1.configNominalOutputReverse(0,kTimeoutMs);
        c2.configNominalOutputReverse(0,kTimeoutMs);

        c1.configPeakOutputForward(1,kTimeoutMs);
        c2.configPeakOutputForward(1,kTimeoutMs);

        c1.configPeakOutputReverse(-1,kTimeoutMs);
        c2.configPeakOutputReverse(-1,kTimeoutMs);

        c1.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
        c2.selectProfileSlot(kSlotIdx, kPIDLoopIdx);

        c1.config_kF(kSlotIdx,kf,kTimeoutMs);
        c2.config_kF(kSlotIdx,kf,kTimeoutMs);

        c1.config_kP(kSlotIdx,kp,kTimeoutMs);
        c2.config_kP(kSlotIdx,kp,kTimeoutMs);

        c1.config_kI(kSlotIdx,ki,kTimeoutMs);
        c2.config_kI(kSlotIdx,ki,kTimeoutMs);

        c1.config_kD(kSlotIdx,kd,kTimeoutMs);
        c2.config_kD(kSlotIdx,kd,kTimeoutMs);

        c1.configMotionCruiseVelocity(15000, kTimeoutMs);
        c2.configMotionCruiseVelocity(15000, kTimeoutMs);
        c1.configMotionAcceleration(6000, kTimeoutMs);
        c2.configMotionAcceleration(6000, kTimeoutMs);

        c1.setSelectedSensorPosition(0, kPIDLoopIdx,kTimeoutMs);
        c2.setSelectedSensorPosition(0, kPIDLoopIdx,kTimeoutMs);
    }
}