/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//I HAVE NO IDEA WHY THIS ISN'T ACCEPTING THE com and ctre LIBRARIES 10/1/19
//Looks to be working 10/10/19
//import edu.wpi.first.wpilibj.VictorSPX;
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
    public static final double RESET_DELAY_SEC = 0.25d;
    public static final double RETRACT_TIME_SEC = 0.25d;

    /*
    fireVent
       |
    V--+--fireValve---+
    V                 +--Tank--Compressor
    V--+--resetValve--+
       |
    resetVent
    */
    Compressor compressor = new Compressor(0);
    Solenoid fireValve = new Solenoid(1); // F
    Solenoid fireVent = new Solenoid(2); // FV
    Solenoid resetValve = new Solenoid(3); // R
    Solenoid resetVent = new Solenoid(4); // RV

    private class PMState{
        float stateTime;
        boolean fireValve;
        boolean fireVent;
        boolean resetValve;
        boolean resetVent;
        public PMState(float stateTime, boolean f, boolean fv, boolean r, boolean rv)
        {
            this.stateTime = stateTime;
            fireValve = f;
            fireVent = fv;
            resetValve = r;
            resetVent = rv;
        }
    }
    // Pneumatics: false=closed, true=open
    private final PMState pmStandby = new PMState(100, false, false, true, false); // The default state, when not in a firing cycle.
    private int pmSeqIdx = 0; // Index in pmFireSequence that I'm currently on
    private final PMState[] pmFireSequence = {
        new PMState(0.25f, true, false, false, true), // Fire
        new PMState(0.1f, false, false, false, false), // pre Reset
        new PMState(0.25f, false, true, true, false), // Reset
        new PMState(0.1f, false, false, false, false) // pre Standby
    };
    private PMState fireState = pmStandby; // Always use setFireState() to change this!
    /** Returns true if the piston has been fired and cannot shoot until it retracts. Returns false otherwise. */
    //public PMState getFireState() { return fireState; }
    public void setFireState(PMState value) {
        fireState = value;
        timeAtLastStateChange = Timer.getFPGATimestamp();
    }
    private double timeAtLastStateChange = 0;
    public double timeSinceStateChange() {
        return Timer.getFPGATimestamp() - timeAtLastStateChange;
    }

    public void setDriver(double var[]){
        VictorSPX motorRT = new VictorSPX(0);
        VictorSPX motorLT = new VictorSPX(1);
        VictorSPX motorRB = new VictorSPX(2);
        VictorSPX motorLB = new VictorSPX(3);
		
		
    }
    
    public void fire()
    {
        if(fireState != pmStandby)
            return; // Can't fire before piston has been reset! TODO: give a warning message here?

        // Start the firing sequence by moving off of pmStandby:
        pmSeqIdx = 0;
        setFireState(pmFireSequence[pmSeqIdx]);
    }

    /**
     * Called once each autonomousPeriodic() and teleopPeriodic() step by Robot.java.
     */
    public void enabledPeriodic()
    {
        // In in a fire cycle, set solenoids to appropriate states and check for moving to the next step
        if(fireState != pmStandby)
        {
            fireValve.set(fireState.fireValve);
            fireVent.set(fireState.fireVent);
            resetValve.set(fireState.resetValve);
            resetVent.set(fireState.resetVent);

            if(timeSinceStateChange() > fireState.stateTime) // Time to move to the next step?
            {
                pmSeqIdx++;
                if(pmSeqIdx < pmFireSequence.length)
                    setFireState(pmFireSequence[pmSeqIdx]);
                else
                    setFireState(pmStandby);
            }
        }
    }
}
