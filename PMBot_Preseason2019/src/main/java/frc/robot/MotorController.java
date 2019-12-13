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
import com.revrobotics.CANEncoder;
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

    public static final double RESET_DELAY_SEC = 0.25d;
    public static final double RETRACT_TIME_SEC = 0.25d;
    private static final int leadDeviceID = 1;
    private CANSparkMax m_leadMotor = new CANSparkMax(leadDeviceID, MotorType.kBrushless);
    private CANEncoder m_encoder;
    private CANPIDController leadPIDController;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM; // PID coefficients

    /*
    resetVent
       |
    V--+--+--fireValve1--+
    V     +--fireValve2--+
    V                    +--Tank--Compressor
    V-----resetValve-----+
    */
    Solenoid fireValve1 = new Solenoid(1); // F1
    Solenoid fireValve2 = new Solenoid(2); // F2
    Solenoid resetValve = new Solenoid(3); // R 3-port solenoid; doubles as fire vent (venting when unpowered)
    Solenoid resetVent = new Solenoid(5); // RV

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
        maxRPM = 5700;
        // set PID coefficients
        leadPIDController.setP(kP);
        leadPIDController.setI(kI);
        leadPIDController.setD(kD);
        leadPIDController.setIZone(kIz);
        leadPIDController.setFF(kFF);
        leadPIDController.setOutputRange(kMinOutput, kMaxOutput);
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);


        m_leadMotor.restoreFactoryDefaults(); // TODO: run these on teleop begin or whatever
    }

    public void setSparkTest(double var[]) // Call duting Robot.teleopPeriodic()
    {
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double rotations = SmartDashboard.getNumber("Set Rotations", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { leadPIDController.setP(p); kP = p; }
        if((i != kI)) { leadPIDController.setI(i); kI = i; }
        if((d != kD)) { leadPIDController.setD(d); kD = d; }
        if((iz != kIz)) { leadPIDController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { leadPIDController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
            leadPIDController.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max; 
        }

        /**
         * PIDController objects are commanded to a set point using the 
         * SetReference() method.
         * 
         * The first parameter is the value of the set point, whose units vary
         * depending on the control type set in the second parameter.
         * 
         * The second parameter is the control type can be set to one of four 
         * parameters:
         *  com.revrobotics.ControlType.kDutyCycle
         *  com.revrobotics.ControlType.kPosition
         *  com.revrobotics.ControlType.kVelocity
         *  com.revrobotics.ControlType.kVoltage
         */
        rotations = var[0]*maxRPM;
        leadPIDController.setReference(rotations, ControlType.kVelocity);
        
        SmartDashboard.putNumber("SetPoint", rotations);
        SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
    }

    public void setMax(double var[], boolean[] setDist){
        long startTime = System.currentTimeMillis();
        m_encoder = m_leadMotor.getEncoder();

        SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());  //Shows the position of the motorcontroller
        SmartDashboard.putNumber("Encoder Velocity", m_encoder.getVelocity());  //Shows the velocity of the motorcontroller
        SmartDashboard.putBoolean("Encoder?", setDist[0]);
        //Start encoder-based moving
        
        long timeNow = System.currentTimeMillis();
        timeNow = timeNow - startTime;
        //System.out.println(timeNow);

        if(setDist[0] && !setDist[1] && timeNow <= 5000){

            if(setDist[0] && !setDist[1] && m_encoder.getPosition() < 9){
                m_leadMotor.set(0.15);

            } if(setDist[0] && !setDist[1] && 9 < m_encoder.getPosition() && m_encoder.getPosition() < 9.5){
                m_leadMotor.set(0.05);

            } if(setDist[0] && !setDist[1] && 9.95 < m_encoder.getPosition() && m_encoder.getPosition() < 10){
                m_leadMotor.set(0.025);

            } if(setDist[0] && !setDist[1] && 9.5 < m_encoder.getPosition() && m_encoder.getPosition() < 9.95){
                m_leadMotor.set(0.005);

            }
            
            if(setDist[0] && !setDist[1] && 10 < m_encoder.getPosition() && m_encoder.getPosition() < 10.09){
                m_leadMotor.set(-0.005);

            } if(setDist[0] && !setDist[1] && 10.09 < m_encoder.getPosition() && m_encoder.getPosition() < 10.5){
                m_leadMotor.set(-0.023);

            } if(setDist[0] && !setDist[1] && 10.5 < m_encoder.getPosition() && m_encoder.getPosition() < 10.8){
                m_leadMotor.set(-0.045);

            } if(setDist[0] && !setDist[1] && m_encoder.getPosition() > 10.8){
                m_leadMotor.set(-0.15);

            }
        } else if(setDist[0] && !setDist[1] && m_encoder.getPosition() > 9.98 && m_encoder.getPosition() < 10.05){
            m_leadMotor.set(0);

        } else if(!setDist[0] && !setDist[1]){
            m_leadMotor.set(var[0]);
        }

        //Since RevRobotics doesn't have a Encoder resetting function, I'm trying to make one here...?

        if(!setDist[0] && setDist[1]){ //Moves the 
            m_encoder.setPosition(0);
        }
    }

    //    ------------------ BEGIN MECANUM WHEEL CODE ------------------
    public void driCartesian(double[] axis){
        VictorSPX driveRT = new VictorSPX(3); // Right is 3 1
        VictorSPX driveRB = new VictorSPX(1);
        VictorSPX driveLT = new VictorSPX(4); // Left is 4 2
        VictorSPX driveLB = new VictorSPX(2);

        if(axis[0] < -0.2 || axis[0] > 0.2 && axis[1] == 0 && axis[2] == 0){    //Move forward/backward
            driveLT.set(ControlMode.PercentOutput, axis[0]);
            driveLB.follow(driveLT);
            driveRT.set(ControlMode.PercentOutput, -axis[0]);
            driveRB.follow(driveRT);
        } else if(axis[1] > 0.2 && axis[0] == 0 && axis[2] == 0){ //Strafe left
            driveLT.set(ControlMode.PercentOutput, Math.abs(axis[1]));
            driveLB.set(ControlMode.PercentOutput, Math.abs(axis[1]));
            driveRT.set(ControlMode.PercentOutput, Math.abs(axis[1]));
            driveRB.set(ControlMode.PercentOutput, Math.abs(axis[1]));
        } else if(axis[1] < -0.2 && axis[0] == 0 && axis[2] == 0){  //Strafe right
            driveLT.set(ControlMode.PercentOutput, Math.abs(axis[1]));
            driveLB.set(ControlMode.PercentOutput, -Math.abs(axis[1]));
            driveRT.set(ControlMode.PercentOutput, Math.abs(axis[1]));
            driveRB.set(ControlMode.PercentOutput, -Math.abs(axis[1]));
        } else if(axis[2] > 0.1 && axis[0] == 0 && axis[1] == 0){   //Turn clockwise
            driveLT.set(ControlMode.PercentOutput, -axis[2]);
            driveLB.set(ControlMode.PercentOutput, -axis[2]);
            driveRT.set(ControlMode.PercentOutput, -axis[2]);
            driveRB.set(ControlMode.PercentOutput, -axis[2]);
        } else if(axis[2] < -0.1 && axis[0] == 0 && axis[1] == 0){  //Turn counterclockwise
            driveLT.set(ControlMode.PercentOutput, -axis[2]);
            driveLB.set(ControlMode.PercentOutput, -axis[2]);
            driveRT.set(ControlMode.PercentOutput, -axis[2]);
            driveRB.set(ControlMode.PercentOutput, -axis[2]);
        }
    }
    



    //   --------------------   BEGIN PNEUMATICS CODE   --------------------
    private static class PMState{
        float stateTime;
        boolean fireValve1;
        boolean fireValve2;
        boolean resetValve;
        boolean resetVent;
        public PMState(float stateTime, boolean f1, boolean f2, boolean r, boolean rv)
        {
            this.stateTime = stateTime;
            fireValve1 = f1;
            fireValve2 = f2;
            resetValve = r;
            resetVent = rv;
        }
    }
    // Pneumatics: false=closed, true=open
    private final PMState pmStandby = new PMState(100, false, false, false, false); // The default state, when not in a firing cycle.
    private int pmSeqIdx = 0; // Index in pmFireSequence that I'm currently on
    private final PMState[] pmFireSequence = {
        new PMState(0.25f, true, true, false, false), // Fire
        new PMState(0.1f, false, false, false, false), // pre Reset
        new PMState(0.25f, false, false, true, true) // Reset
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
        //System.out.println(pmSeqIdx);

        // In in a fire cycle, set solenoids to appropriate states and check for moving to the next step
        if(fireState != pmStandby)
        {
            if(timeSinceStateChange() > fireState.stateTime) // Time to move to the next step?
            {
                //System.out.println("next step");
                pmSeqIdx++;
                if(pmSeqIdx < pmFireSequence.length)
                    setFireState(pmFireSequence[pmSeqIdx]);
                else
                {
                    setFireState(pmStandby);
                    //System.out.println("standby reset");
                    }
            }
        }

        fireValve1.set(fireState.fireValve1);
        fireValve2.set(fireState.fireValve2);
        resetValve.set(fireState.resetValve);
        resetVent.set(fireState.resetVent);

        
    }
}
