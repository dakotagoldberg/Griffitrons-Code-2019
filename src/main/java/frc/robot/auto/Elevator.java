/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
/**
 * Add your docs here.
 */
public class Elevator {

    private WPI_TalonSRX e1 = new WPI_TalonSRX(6);
    private WPI_TalonSRX e2 = new WPI_TalonSRX(7);
    private WPI_TalonSRX e3 = new WPI_TalonSRX(8);
    private WPI_TalonSRX e4 = new WPI_TalonSRX(9);

    int height1=42,height2=42,height3=42, storageHeight=42, groundHeight=42;

    public void toHeight1(){
        //takes elevator to height of choice
    }
    public void toHeight2(){
        //takes elevator to height of choice
    }
    public void toHeight3(){
        //takes elevator to height of choice
    }
    public void toStorageHeight(){
        //takes elevator to height of choice
    }
    public void toGroundHeight(){
        //takes elevator to height of choice
    }
    public void zeroElevator(){
        //zeros the elevator location
    }
}