// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class DavePoint {

    public double x, y, theta, weight, vel, head;
    public double[] segpoint;

    public DavePoint(double x, double y, double theta, double weight, double vel, double head){
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.weight = weight;
        this.vel = vel;
        this.head = head;
        segpoint = new double[]{x, y, theta, weight};
    }

    public DavePoint(){
        this(0, 0);
    }

    public DavePoint(double x, double y){
        this(x, y, 0, 0, 0, 1);
    }

    public double radiansFrom(double x0, double y0){
        //if(x - x0 == 0.0) return Math.PI / 2;
        return Math.atan2(y - y0, x - x0);
    }

}
