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

import com.revrobotics.CANSparkMax;
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
    private CANSparkMax m_leadMotor = new CANSparkMax(leadDeviceID, MotorType.kBrushless);
    private CANSparkMax m_followMotor = new CANSparkMax(followDeviceID, MotorType.kBrushless);
    private VictorSPX driveR1 = new VictorSPX(3); // Right is 3 4
    private VictorSPX driveR2 = new VictorSPX(4);
    private VictorSPX driveL1 = new VictorSPX(1); // Left is 1 2
    private VictorSPX driveL2 = new VictorSPX(2);
    public static final double RESET_DELAY_SEC = 0.25d;
    public static final double RETRACT_TIME_SEC = 0.25d;
    private static final int leadDeviceID = 1;
    private static final int followDeviceID = 2;

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

    private double x;   //Testing Mecanum code from another team on GitHub 10/22/19
    private double y;   //Testing Mecanum code from another team on GitHub 10/22/19
    private double z;   //Testing Mecanum code from another team on GitHub 10/22/19

    public MotorController()
    {
        driveR2.follow(driveR1);
        driveL2.follow(driveL1);
    }

    public enum FireState {
        Primed,     // FV open
        PreFire,    // RV open
        Fire,        // F, RV open
        Extended,    // RV open
        PreRetract,  // FV open
        Retract    // R, FV open
    }
    private FireState fireState = FireState.Primed;
    /** Returns true if the piston has been fired and cannot shoot until it retracts. Returns false otherwise. */
    public FireState getFireState() { return fireState; }
    public void setFireState(FireState value) {
        fireState = value;
        timeAtLastStateChange = Timer.getFPGATimestamp();
    }
    private double timeAtLastStateChange = 0;
    public double timeSinceStateChange() {
        return Timer.getFPGATimestamp() - timeAtLastStateChange;
    }

    public void setDriver(double var[]){
       m_leadMotor.restoreFactoryDefaults(); // TODO: run these on teleopbegin or whatever
       m_followMotor.restoreFactoryDefaults();
       m_followMotor.follow(m_leadMotor);

       m_leadMotor.set(var[0]);

        driveR1.set(ControlMode.Velocity, var[0]);
        driveL1.set(ControlMode.Velocity, var[1]);
    }
    
    public void fire()
    {
        if(fireState != FireState.Primed)
            return; // Can't fire before piston has been reset! TODO: give a warning message here?

        fireValve.set(true);
        resetValve.set(false);
        //setFireState(FireState.Firing);
    }

    /**
     * Called once each teleopPeriodic() step.
     */
    public void teleopPeriodic()
    {
        /*if(fireState==FireState.Firing && timeSinceStateChange() > RESET_DELAY_SEC)
        {
            fireValve.set(false);
            resetValve.set(true);
            setFireState(FireState.Firing);
        }*/
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
