/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Add your docs here.
 */
public class DaveDigitalInput extends DigitalInput{
    private boolean previous_value;
    private boolean current_value;
    
    public DaveDigitalInput (int port){
        super(port);
        previous_value = get();
        current_value = get();
    }

    public void update(){
        previous_value = current_value;
        current_value = get();
    }

    public boolean getTriggered(){
        return !current_value && previous_value;
    }

    public boolean getReleased(){
        return current_value && !previous_value;
    }
}