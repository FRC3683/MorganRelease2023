// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class GlobalTimer {

    private static GlobalTimer instance;
    public static GlobalTimer getInstance(){
        if(instance == null) instance = new GlobalTimer();
        return instance;
    }

    Timer timer;
    boolean active;

    private GlobalTimer(){
        timer = new Timer();
        active = false;
    }

    public void reset(){
        timer.reset();
    }

    public void start(){
        timer.start();
        active = true;
    }

    public void stop(){
        timer.stop();
        active = false;
    }

    public double get(){
        return timer.get();
    }

    public boolean isActive(){
        return active;
    }
}
