/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import frc.robot.constants.Robot_Framework;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

/**
 * Add your docs here.
 */
public class Elevator implements Robot_Framework {

    private static final int kPIDLoopIdx = 0;
    private static final int kSlotIdx = 0;

    public void placeHatch(int level) {
        if(level == 0) {
            leftElev.set(ControlMode.MotionMagic, left_elev_hatch_bottom);
            rightElev.set(ControlMode.MotionMagic, right_elev_hatch_bottom);
        } else if(level == 1) {
            leftElev.set(ControlMode.MotionMagic, left_elev_hatch_middle);
            rightElev.set(ControlMode.MotionMagic, right_elev_hatch_middle);
        } else {
            leftElev.set(ControlMode.MotionMagic, left_elev_hatch_top);
            rightElev.set(ControlMode.MotionMagic, right_elev_hatch_top);
        }
    }

    public void placeBall(int level) {
        if(level == 0) {
            leftElev.set(ControlMode.MotionMagic, left_elev_ball_bottom);
            rightElev.set(ControlMode.MotionMagic, right_elev_ball_bottom);
        } else if(level == 1) {
            leftElev.set(ControlMode.MotionMagic, left_elev_ball_middle);
            rightElev.set(ControlMode.MotionMagic, right_elev_ball_middle);
        } else {
            leftElev.set(ControlMode.MotionMagic, left_elev_ball_top);
            rightElev.set(ControlMode.MotionMagic, right_elev_ball_top);
        }
    }

    public Elevator(){
        leftElev.configFactoryDefault();
        rightElev.configFactoryDefault();

        leftElev.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
        rightElev.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);

        leftElev.setSensorPhase(false);
        rightElev.setSensorPhase(false);
        
        leftElev.setInverted(false);
        rightElev.setInverted(false);

        leftElev.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        rightElev.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        leftElev.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);
        rightElev.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

        leftElev.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
        rightElev.selectProfileSlot(kSlotIdx, kPIDLoopIdx);

        leftElev.config_kP(kSlotIdx, elev_p, kTimeoutMs);
        rightElev.config_kP(kSlotIdx, elev_p, kTimeoutMs);

        leftElev.config_kI(kSlotIdx, elev_i, kTimeoutMs);
        rightElev.config_kI(kSlotIdx, elev_i, kTimeoutMs);

        leftElev.config_kD(kSlotIdx, elev_d, kTimeoutMs);
        rightElev.config_kD(kSlotIdx, elev_d, kTimeoutMs); 

        leftElev.config_kF(kSlotIdx, elev_f, kTimeoutMs);
        rightElev.config_kF(kSlotIdx, elev_f, kTimeoutMs); 

        leftElev.configMotionCruiseVelocity(elev_cruise_velocity);
        rightElev.configMotionCruiseVelocity(elev_cruise_velocity);

        leftElev.configMotionAcceleration(elev_cruise_accel);
        rightElev.configMotionAcceleration(elev_cruise_accel);
    }
}