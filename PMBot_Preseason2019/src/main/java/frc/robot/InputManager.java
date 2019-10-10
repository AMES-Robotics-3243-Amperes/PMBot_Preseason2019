/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Add your docs here.
 */
public class InputManager {
    
    Joystick inputOne = new Joystick(0);
    private static final double JOY_CURVE_EXP = 2;
    private static final double JOY_DEAD_THRESHOLD = 0.1d;

    /**
     * Curves and deadzones a joystick axis value
     * @param rawValue
     * @return
     */
    private double processJoyst(double rawValue)
    {
        return Math.pow((rawValue-JOY_DEAD_THRESHOLD) / (1-JOY_DEAD_THRESHOLD), JOY_CURVE_EXP);
    }

    public double[] throttles()
    {
        return new double[] {
            - processJoyst(inputOne.getRawAxis(1)),
            - processJoyst(inputOne.getRawAxis(0))
            
        };
    }

}
