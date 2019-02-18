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
public class IntakeRotate implements Robot_Framework {

    private static final int kPIDLoopIdx = 0;
    private static final int kSlotIdx = 0;

    public void ballMode() {
        iRotate.config_kP(kSlotIdx, dash.getP(), kTimeoutMs);
        iRotate.config_kI(kSlotIdx, dash.getI(), kTimeoutMs);
        iRotate.config_kD(kSlotIdx, dash.getD(), kTimeoutMs);
        iRotate.set(ControlMode.MotionMagic, intake_rotate_ball);
    }

    public void hatchMode() {
        iRotate.config_kP(kSlotIdx, dash.getP(), kTimeoutMs);
        iRotate.config_kI(kSlotIdx, dash.getI(), kTimeoutMs);
        iRotate.config_kD(kSlotIdx, dash.getD(), kTimeoutMs);
        iRotate.set(ControlMode.MotionMagic, intake_rotate_hatch);
    }

    public IntakeRotate(){
        iRotate.configFactoryDefault();

        iRotate.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);

        iRotate.setSensorPhase(false);
        
        iRotate.setInverted(false);

        iRotate.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        iRotate.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

        iRotate.selectProfileSlot(kSlotIdx, kPIDLoopIdx);

        iRotate.config_kF(kSlotIdx, intake_rotate_f, kTimeoutMs);

        iRotate.configMotionCruiseVelocity(intake_rotate_cruise_velocity);

        iRotate.configMotionAcceleration(intake_rotate_cruise_accel);
    }
}