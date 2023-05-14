package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;

public class TwoStagePID {
    private PIDController pidFar;
    private PIDController pidClose;
    private double threshhold;
    private double cutoffStart, cutoffEnd; // cutoffEnd meant to be greater than cutoffStart
    private boolean running;
    private boolean useMinMaxOutput;
    private double minOutput, maxOutput;
    private Debouncer debouncer;

    public TwoStagePID(PIDController pidFar, PIDController pidClose, double threshhold, double cutoffStart, double cutoffEnd) {
        this.pidFar = pidFar;
        this.pidClose = pidClose;
        this.threshhold = threshhold;
        this.cutoffStart = cutoffStart;
        this.cutoffEnd = cutoffEnd;
        pidFar.setTolerance(cutoffEnd);
        pidClose.setTolerance(cutoffEnd);
        running = true;
        useMinMaxOutput = false;
        debouncer = new Debouncer(0);
    }

    public TwoStagePID debounceThreshhold(double debounceTime) {
        debouncer = new Debouncer(debounceTime);
        return this;
    }

    public TwoStagePID enableMinMaxOutput(double minOutput, double maxOutput) {
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        useMinMaxOutput = true;
        return this;
    }

    public boolean atReference(double measurement) {
        if(Math.abs(measurement) > threshhold) {
            return pidFar.atSetpoint();
        }
        return pidClose.atSetpoint();
    }

    public TwoStagePID enableContinuousOutput(double min, double max) {
        pidFar.enableContinuousInput(min, max);
        pidClose.enableContinuousInput(min, max);
        return this;
    }

    private void check(double error) {
        error = Math.abs(error);
        if(error < cutoffStart) {
            running = false;
        }
        if(error > cutoffEnd) {
            running = true;
        }
    }

    public double calculate(double measurement) {
        check(measurement);
        if(!running) {
            return 0;
        }
        double value = 0;
        if(debouncer.calculate(Math.abs(measurement) < threshhold)) {
            value = pidClose.calculate(measurement);
        } else {
            value = pidFar.calculate(measurement);
        }
        return useMinMaxOutput ? MathUtils.clamp(minOutput, maxOutput, value) : value;
    }

    public double calculate(double current, double setpoint) {
        double diff = setpoint - current;
        check(diff);
        if(!running) {
            return 0;
        }
        double value = 0;
        if(debouncer.calculate(Math.abs(diff) < threshhold)) {
            value = pidClose.calculate(current, setpoint);
        } else {
            value = pidFar.calculate(current, setpoint);
        }
        return useMinMaxOutput ? MathUtils.clamp(minOutput, maxOutput, value) : value;
    }
}
