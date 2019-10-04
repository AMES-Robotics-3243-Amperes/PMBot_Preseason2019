/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//I HAVE NO IDEA WHY THIS ISN'T ACCEPTING THE com and ctre LIBRARIES 10/1/19
import edu.wpi.first.wpilibj.VictorSP;
import com.ctre.phoenix.ILoopable;
import com.ctre.poenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * Add your docs here.
 */
public class MotorController {

    VictorSPX motorRT = new VictorSPX(0);
    VictorSPX motorLT = new VictorSPX(1);
    VictorSPX motorRB = new VictorSPX(2);
    VictorSPX motorLB = new VictorSPX(3);

    public void setDriver(double var[]){
        //Driving straight. I could be wrong on this: 10/3/19
        motorRT.set(ControlMode.PercentOutput,  var[0]);
        motorLT.set(ControlMode.PercentOutput, -var[1]);
        motorRB.set(ControlMode.PercentOutput,  var[0]);
        motorLB.set(ControlMode.PercentOutput, -var[1]);

    }
    
}
