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
import edu.wpi.first.wpilibj.drive.MecanumDrive;    //Testing Mecanum code from another team on GitHub 10/22/19
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.ILoopable;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * Add your docs here.
 */
public class MotorController {
    // 2019 Nov 7: VSCode intermittently stops recognizing Spark libraries

    private VictorSPX driveR1 = new VictorSPX(3); // Right is 3 4
    private VictorSPX driveR2 = new VictorSPX(4);
    private VictorSPX driveL1 = new VictorSPX(1); // Left is 1 2
    private VictorSPX driveL2 = new VictorSPX(2);
    public static final double RESET_DELAY_SEC = 0.25d;
    public static final double RETRACT_TIME_SEC = 0.25d;
    private static final int leadDeviceID = 1;
    private static final int followDeviceID = 2;
    private CANSparkMax m_leadMotor = new CANSparkMax(leadDeviceID, MotorType.kBrushless);
    private CANSparkMax m_followMotor = new CANSparkMax(followDeviceID, MotorType.kBrushless);
    private CANEncoder m_encoder;
    private CANPIDController leadPIDController;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput; // PID coefficients

    m_leadMotor.restoreFactoryDefaults(); // TODO: run these on teleop begin or whatever
    m_followMotor.restoreFactoryDefaults();
    //private MecanumDrive robotDrive = new MecanumDrive(driveL1, driveL2, driveR1, driveR2);

    /*
    fireVent
       |
    V--+--fireValve---+
    V                 +--Tank--Compressor
    V--+--resetValve--+
       |
    resetVent
    */
    Solenoid fireValve = new Solenoid(1); // F
    Solenoid fireVent = new Solenoid(2); // FV
    Solenoid resetValve = new Solenoid(3); // R
    Solenoid resetVent = new Solenoid(4); // RV

    private double x;   //Testing Mecanum code from another team on GitHub 10/22/19
    private double y;   //Testing Mecanum code from another team on GitHub 10/22/19
    private double z;   //Testing Mecanum code from another team on GitHub 10/22/19
    
    public MotorController() // Constructed in Robot.robotInit()
    {
        leadPIDController = m_leadMotor.getPIDController();
        // PID coefficients
        kP = 0.1; 
        kI = 1e-4;
        kD = 1; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        // set PID coefficients
        leadPIDController.setP(kP);
        leadPIDController.setI(kI);
        leadPIDController.setD(kD);
        leadPIDController.setIZone(kIz);
        leadPIDController.setFF(kFF);
        leadPIDController.setOutputRange(kMinOutput, kMaxOutput);

        driveR2.follow(driveR1);
        driveL2.follow(driveL1);
    }

    public void setSparkTest() // Call duting Robot.teleopPeriodic()
    {
        leadPIDController.setReference(10, ControlType.kPosition);
    }

    //   --------------------   BEGIN PNEUMATICS CODE   --------------------
    private static class PMState{
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

    public double setMax(double var[]){
       m_leadMotor.set(var[0]);
       m_encoder = m_leadMotor.getEncoder();
       SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
       SmartDashboard.putNumber("Encoder Velocity", m_encoder.getVelocity());
    }
    
    public void setDriver(double var[]){    //Mecanum Wheels

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

    public void setX(double x){ //Testing Mecanum code from another team on GitHub 10/22/19
        this.x = x;
    }
    public void setY(double y){ //Testing Mecanum code from another team on GitHub 10/22/19
        this.y = y;
    }
    public void setZ(double z){ //Testing Mecanum code from another team on GitHub 10/22/19
        this.z = z;
    }

}
