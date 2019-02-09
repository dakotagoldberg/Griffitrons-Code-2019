
package frc.robot.auto;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
    
/**
 * Add your docs here.
 */
public class Drive {
    private WPI_TalonSRX L1 = new WPI_TalonSRX(0);
    private WPI_TalonSRX L2 = new WPI_TalonSRX(1);
    private WPI_TalonSRX L3 = new WPI_TalonSRX(2);
    private WPI_TalonSRX R1 = new WPI_TalonSRX(3);
    private WPI_TalonSRX R2 = new WPI_TalonSRX(4);
    private WPI_TalonSRX R3 = new WPI_TalonSRX(5);

    public void autoPath(int path){
        //int fed from dash board determines path then it follows apropriate tape
    }
    public void setpoint(int x, int y, int theta){
        //give it a point and it goes there, theta for rotation
    }
    public void setpointArray(int[] cords){
        //set point but it recieves array from GetGoal;
    }

}
