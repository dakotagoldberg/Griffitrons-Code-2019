/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.possibleclasses;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
/**
 * Add your docs here.
 */
public class Intake {

    private WPI_TalonSRX r1 = new WPI_TalonSRX(10);
    private WPI_TalonSRX r2 = new WPI_TalonSRX(11);
    private WPI_TalonSRX h1 = new WPI_TalonSRX(12);
    private WPI_TalonSRX h2 = new WPI_TalonSRX(13);
    private WPI_TalonSRX rotation = new WPI_TalonSRX(14);
    private int h1zero = 42, h2zero = 42;

    public void ballIntake(){
        //turns rolllers and such to intake ball
    }
    public void ballShoot(){
        //turns rolllers and such to shoot ball
    }
    public void hatchIntake(){
        //turns rolllers and such to intake hatch
    }
    public void hatchShoot(){
        //turns rolllers and such to release hatch
    }
    public void zeroRotation(){
        //zeros the Rotation
    }
    public void rotateTO(int degrees){
        //rotates the input to that degree amount
    }
}