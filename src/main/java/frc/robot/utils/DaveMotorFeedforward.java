// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.LinearSystemId;

/** Add your docs here. */
public class DaveMotorFeedforward extends SimpleMotorFeedforward {

    /**
     * Creates a new DaveMotorFeedforward with the specified gains. Units of the
     * gain values will
     * dictate units of the computed feedforward.
     *
     * @param ks The static gain.
     * @param kv The velocity gain.
     * @param ka The acceleration gain.
     */
    public DaveMotorFeedforward(double ks, double kv, double ka) {
        super(ks, kv, ks);
    }

    /**
     * Creates a new SimpleMotorFeedforward with the specified gains. Acceleration
     * gain is defaulted
     * to zero. Units of the gain values will dictate units of the computed
     * feedforward.
     *
     * @param ks The static gain.
     * @param kv The velocity gain.
     */
    public DaveMotorFeedforward(double ks, double kv) {
        this(ks, kv, 0);
    }

    /**
     * Calculates the stiction feedforward from the gains and setpoints.
     *
     * @param velocity     The velocity setpoint.
     * @param acceleration The acceleration setpoint.
     * @return The computed feedforward.
     */
    public double stiction(double velocity, double acceleration) {
        return ks * Math.signum(velocity);
    }

    /**
     * Calculates the velocity feedforward from the gains and setpoints.
     *
     * @param velocity     The velocity setpoint.
     * @param acceleration The acceleration setpoint.
     * @return The computed feedforward.
     */
    public double velff(double velocity, double acceleration) {
        return kv * velocity;
    }

    /**
     * Calculates the acceleration feedforward from the gains and setpoints.
     *
     * @param velocity     The velocity setpoint.
     * @param acceleration The acceleration setpoint.
     * @return The computed feedforward.
     */
    public double accff(double velocity, double acceleration) {
        return ka * acceleration;
    }

    /**
     * Calculates the stiction feedforward from the gains and setpoints.
     *
     * @param currentVelocity The current velocity setpoint.
     * @param nextVelocity    The next velocity setpoint.
     * @param dtSeconds       Time between velocity setpoints in seconds.
     * @return The computed feedforward.
     */
    public double stiction(double currentVelocity, double nextVelocity, double dtSeconds) {
        return ks * Math.signum(currentVelocity);
    }

    /**
     * Calculates the velocity feedforward from the gains and setpoints.
     *
     * @param currentVelocity The current velocity setpoint.
     * @param nextVelocity    The next velocity setpoint.
     * @param dtSeconds       Time between velocity setpoints in seconds.
     * @return The computed feedforward.
     */
    public double velff(double currentVelocity, double nextVelocity, double dtSeconds) {
        var plant = LinearSystemId.identifyVelocitySystem(this.kv, this.ka);
        var feedforward = new LinearPlantInversionFeedforward<>(plant, dtSeconds);

        var r = Matrix.mat(Nat.N1(), Nat.N1()).fill(currentVelocity);
        var nextR = Matrix.mat(Nat.N1(), Nat.N1()).fill(nextVelocity);

        return feedforward.calculate(r, nextR).get(0, 0);
    }
}
