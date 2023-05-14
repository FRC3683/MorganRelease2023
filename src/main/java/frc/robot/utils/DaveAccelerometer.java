/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class DaveAccelerometer extends BuiltInAccelerometer{
    public static DaveAccelerometer instance;
    public double x_velocity;
    public double y_velocity;
    public double z_velocity;
    private double t;
    private double dt;
    private double last_t;


    public static DaveAccelerometer getInstance(){
        if(instance == null){
            instance = new DaveAccelerometer();
        }
        return instance;
    }

    private DaveAccelerometer(){
        t = 0;
        dt = 0;
        last_t = 0;
        resetVelocities();
    }

    public void resetVelocities(){
        x_velocity = 0;
        y_velocity = 0;
        z_velocity = 0;
    }

    /**
     * @return the x_velocity
     */
    public double getXVelocity() {
        return x_velocity;
    }

    /**
     * @return the y_velocity
     */
    public double getYVelocity() {
        return y_velocity;
    }

    /**
     * @return the z_velocity
     */
    public double getZVelocity() {
        return z_velocity;
    }
    
    public void updateVelocities() {
        t = Timer.getFPGATimestamp();
        dt = t - last_t;
        last_t = t;
        x_velocity += getX() * dt;
        y_velocity += getY() * dt;
        z_velocity += getZ() * dt;
    }
}