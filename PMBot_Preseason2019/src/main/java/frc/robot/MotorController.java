/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//I HAVE NO IDEA WHY THIS ISN'T ACCEPTING THE com and ctre LIBRARIES 10/1/19
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.RobotDrive;
import com.ctre.phoenix.ILoopable;
import com.ctre.poenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * Add your docs here.
 */
public class MotorController {

    public void setDriver(double var[]){
        VictorSP motorRT = new VictorSP(0);
        VictorSP motorLT = new VictorSP(1);
        VictorSP motorRB = new VictorSP(2);
        VictorSP motorLB = new VictorSP(3);
        
    }
    
}
