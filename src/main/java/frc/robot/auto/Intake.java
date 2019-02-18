/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import frc.robot.constants.Robot_Framework;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

/**
 * Add your docs here.
 */
public class Intake implements Robot_Framework {

    private static final int kPIDLoopIdx = 0;
    private static final int kSlotIdx = 0;

    public void ballIntake() {
        // turns rolllers and such to intake ball
    }

    public void ballShoot() {
        // turns rolllers and such to shoot ball
    }

    public void hatchGrab() {
        // turns rolllers and such to intake hatch
        leftClaw.config_kP(kSlotIdx, dash.getP(), kTimeoutMs);
        rightClaw.config_kP(kSlotIdx, dash.getP(), kTimeoutMs);

        leftClaw.config_kI(kSlotIdx, dash.getI(), kTimeoutMs);
        rightIntake.config_kI(kSlotIdx, dash.getI(), kTimeoutMs);

        leftClaw.config_kD(kSlotIdx, dash.getD(), kTimeoutMs);
        rightClaw.config_kD(kSlotIdx, dash.getD(), kTimeoutMs); 

        leftClaw.set(ControlMode.MotionMagic, left_claw_open);
        rightClaw.set(ControlMode.MotionMagic, 319);
    }

    public void hatchRelease() {
        // turns rolllers and such to release hatch
        leftClaw.config_kP(kSlotIdx, dash.getP(), kTimeoutMs);
        rightClaw.config_kP(kSlotIdx, dash.getP(), kTimeoutMs);

        leftClaw.config_kI(kSlotIdx, dash.getI(), kTimeoutMs);
        rightIntake.config_kI(kSlotIdx, dash.getI(), kTimeoutMs);

        leftClaw.config_kD(kSlotIdx, dash.getD(), kTimeoutMs);
        rightClaw.config_kD(kSlotIdx, dash.getD(), kTimeoutMs); 

        leftClaw.set(ControlMode.MotionMagic, left_claw_closed);
        rightClaw.set(ControlMode.MotionMagic, 53);
    }

    public void zeroRotation() {
        // zeros the Rotation
    }

    public void rotateTo(int degrees) {
        // rotates the input to that degree amount
    }

    public Intake(){
        leftClaw.configFactoryDefault();
        rightIntake.configFactoryDefault();

        leftClaw.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, kPIDLoopIdx, kTimeoutMs );
        rightClaw.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, kPIDLoopIdx, kTimeoutMs );

        leftClaw.setSensorPhase(false);
        rightClaw.setSensorPhase(false);
        
        leftClaw.setInverted(false);
        rightClaw.setInverted(false);

        leftClaw.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        rightClaw.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        leftClaw.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);
        rightIntake.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

        leftClaw.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
        rightClaw.selectProfileSlot(kSlotIdx, kPIDLoopIdx);

        leftClaw.config_kF(kSlotIdx, kF, kTimeoutMs);
        rightClaw.config_kF(kSlotIdx, kF, kTimeoutMs); 

        leftClaw.configMotionCruiseVelocity(cruise_velocity);
        rightClaw.configMotionCruiseVelocity(cruise_velocity);

        leftClaw.configMotionAcceleration(cruise_accel);
        rightClaw.configMotionAcceleration(cruise_accel);
    }
}