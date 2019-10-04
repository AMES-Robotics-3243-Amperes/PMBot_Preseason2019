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

    Double [] getDrive(){
        Double [] var = new Double[4];
        //I could be wrong on this: 10/3/19
        var[0] = inputOne.getRawAxis(1);    //Drive forward
        var[1] = -inputOne.getRawAxis(1);   //Drive backwards
        var[2] = inputOne.getRawAxis(0);    //Strafe Right
        var[3] = -inputOne.getRawAxis(0);   //Strafe Left

        return var;
    }

}
