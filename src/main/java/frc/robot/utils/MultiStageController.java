// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;

/** Add your docs here. */
public class MultiStageController {

    interface ControllerCondition {
        boolean condition(double error);
    }

    private ArrayList<Controller> cons;
    private ArrayList<ControllerCondition> conds;
    private double period_s;

    public MultiStageController(double period_s) {
        this.period_s = period_s;
        cons = new ArrayList<>();
        conds = new ArrayList<>();
    }

    public void addCon(double kp, double ki, double kd) {
        addCon(kp, ki, kd, (error) -> {
            return true;
        });
    }

    public void addCon(double kp, double ki, double kd, ControllerCondition cond) {
        addCon(kp, ki, kd, 0.0, 0.0, 0.0, cond);
    }

    public void addCon(double kp, double ki, double kd, double ks, double kv, double ka) {
        addCon(kp, ki, kd, ks, kv, ka, (error) -> {
            return true;
        });
    }

    public void addCon(double kp, double ki, double kd, double ks, double kv, double ka, ControllerCondition cond) {
        cons.add(new Controller(period_s, 0, 0, 0));
        conds.add(cond);
    }

    public void setSetpoint(double setpoint) {
        for (Controller con : cons) {
            con.setSetpoint(setpoint);
        }
    }

    public void enableContinuousInput(double minimumInput, double maximumInput) {
        for (Controller con : cons) {
            con.enableContinuousInput(minimumInput, maximumInput);
        }
    }

    public void reset() {
        for (Controller con : cons) {
            con.reset();
        }
    }

    public double calculate(double measurement) {
        double result = 0.0;

        for (int i = cons.size() - 1; i >= 0; i--) {
            if (conds.get(i).condition(measurement)) {
                // System.out.println("Condition " + i);
                return cons.get(i).calculate(measurement);
            }
        }

        return result;
    }

    public double calculate(double measurement, double next) {
        double result = 0.0;

        for (int i = cons.size() - 1; i >= 0; i--) {
            if (conds.get(i).condition(next - measurement)) {
                return cons.get(i).calculate(measurement, next);
            }
        }

        return result;
    }

}
