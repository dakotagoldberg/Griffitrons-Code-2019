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

    public void hatchGrab() {
        leftClaw.set(ControlMode.MotionMagic, left_claw_open);
        rightClaw.set(ControlMode.MotionMagic, right_claw_open);
    }

    public void hatchRelease() {
        leftClaw.set(ControlMode.MotionMagic, left_claw_closed);
        rightClaw.set(ControlMode.MotionMagic, right_claw_closed);
    }

    public void ballGrab() {
        leftClaw.set(ControlMode.MotionMagic, left_claw_ball);
        rightClaw.set(ControlMode.MotionMagic, right_claw_ball);
    }

    public Intake(){
        leftClaw.configFactoryDefault();
        rightClaw.configFactoryDefault();

        leftClaw.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, kPIDLoopIdx, kTimeoutMs );
        rightClaw.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, kPIDLoopIdx, kTimeoutMs );

        leftClaw.setSensorPhase(false);
        rightClaw.setSensorPhase(false);
        
        leftClaw.setInverted(false);
        rightClaw.setInverted(false);

        leftClaw.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        rightClaw.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        leftClaw.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);
        rightClaw.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

        leftClaw.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
        rightClaw.selectProfileSlot(kSlotIdx, kPIDLoopIdx);

        leftClaw.config_kP(kSlotIdx, claw_p, kTimeoutMs);
        rightClaw.config_kP(kSlotIdx, claw_p, kTimeoutMs);

        leftClaw.config_kI(kSlotIdx, claw_i, kTimeoutMs);
        rightClaw.config_kI(kSlotIdx, claw_i, kTimeoutMs);

        leftClaw.config_kD(kSlotIdx, claw_d, kTimeoutMs);
        rightClaw.config_kD(kSlotIdx, claw_d, kTimeoutMs); 

        leftClaw.config_kF(kSlotIdx, claw_f, kTimeoutMs);
        rightClaw.config_kF(kSlotIdx, claw_f, kTimeoutMs); 

        leftClaw.configMotionCruiseVelocity(claw_cruise_velocity);
        rightClaw.configMotionCruiseVelocity(claw_cruise_velocity);

        leftClaw.configMotionAcceleration(claw_cruise_accel);
        rightClaw.configMotionAcceleration(claw_cruise_accel);
    }
}