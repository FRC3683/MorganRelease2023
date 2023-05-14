// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** Add your docs here. */
public class Controller {

    private PIDController pid;
    private SimpleMotorFeedforward ff;
    private double period_s;

    public Controller(double period_s, double kp, double ki, double kd, double ks, double kv, double ka) {
        this.period_s = period_s;
        pid = new PIDController(kp, ki, kd, period_s);
        ff = new SimpleMotorFeedforward(ks, kv, ka);
    }

    public Controller(double period_s, double kp, double ki, double kd) {
        this(period_s, kp, ki, kd, 0.0, 0.0, 0.0);
    }

    public void setSetpoint(double setpoint) {
        pid.setSetpoint(setpoint);
    }

    public void enableContinuousInput(double minimumInput, double maximumInput) {
        pid.enableContinuousInput(minimumInput, maximumInput);
    }

    public void reset() {
        pid.reset();
    }

    public double calculate(double measurement) {
        return ff.calculate(measurement) + pid.calculate(measurement);
    }

    public double calculate(double measurement, double next) {
        return ff.calculate(measurement, next, period_s) + pid.calculate(measurement, next);
    }

}
