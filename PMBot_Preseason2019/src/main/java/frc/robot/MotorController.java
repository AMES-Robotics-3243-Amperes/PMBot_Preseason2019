/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//I HAVE NO IDEA WHY THIS ISN'T ACCEPTING THE com and ctre LIBRARIES 10/1/19
//Looks to be working 10/10/19
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.ILoopable;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * Add your docs here.
 */
public class MotorController {
    public static final double RESET_DELAY_SECONDS = 0.25d;

    Compressor compressor = new Compressor(0);
    Solenoid fireValve = new Solenoid(1);
    Solenoid resetValve = new Solenoid(2);

    private boolean hasFired = false;
    /** Returns true if the piston has been fired and cannot shoot until it retracts. Returns false otherwise. */
    public boolean getHasFired() { return hasFired; }
    private double timeWhenLastFired = 0;

    public void setDriver(double var[]){
        VictorSP motorRT = new VictorSP(0);
        VictorSP motorLT = new VictorSP(1);
        VictorSP motorRB = new VictorSP(2);
        VictorSP motorLB = new VictorSP(3);
        
    }
    
    public void fire()
    {
        if(hasFired)
            return; // Can't fire before piston has been reset! TODO: give a warning message here?

        fireValve.set(true);
        hasFired = true;
        timeWhenLastFired = Timer.getFPGATimestamp();
    }

    /**
     * Called once each teleopPeriodic() step.
     */
    public void upkeep()
    {
        if(Timer.getFPGATimestamp() - timeWhenLastFired > RESET_DELAY_SECONDS)
    }
}
