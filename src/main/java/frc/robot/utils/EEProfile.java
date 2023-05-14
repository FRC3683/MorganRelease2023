// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.utils.Scurvy.Constraints;

/** Add your docs here. */
public class EEProfile {

    public static class EEConstraints {

        public final double escVelMax_mps;
        public final double lnxVelMax_mps;
        public final double wristVelMax_degps;
        public final double escAccMax_mps;
        public final double lnxAccMax_mps;
        public final double wristAccMax_degps;
        public final double escJrkMax_mps;
        public final double lnxJrkMax_mps;
        public final double wristJrkMax_degps;

        /**
         * Construct constraints for a TrapezoidProfile.
         *
         * @param maxVelocity     maximum velocity
         * @param maxAcceleration maximum acceleration
         */
        public EEConstraints(double escVelMax_mps, double lnxVelMax_mps, double wristVelMax_degps,
                double escAccMax_mps, double lnxAccMax_mps, double wristAccMax_degps,
                double escJrkMax_mps, double lnxJrkMax_mps, double wristJrkMax_degps) {
            this.escVelMax_mps = escVelMax_mps;
            this.lnxVelMax_mps = lnxVelMax_mps;
            this.wristVelMax_degps = wristVelMax_degps;
            this.escAccMax_mps = escAccMax_mps;
            this.lnxAccMax_mps = lnxAccMax_mps;
            this.wristAccMax_degps = wristAccMax_degps;
            this.escJrkMax_mps = escJrkMax_mps;
            this.lnxJrkMax_mps = lnxJrkMax_mps;
            this.wristJrkMax_degps = wristJrkMax_degps;
        }

        @Override
        public String toString() {
            return "evel: " + escVelMax_mps + ", lvel: " + lnxVelMax_mps + ", wvel: " + wristVelMax_degps
                    + ", eacc: " + escAccMax_mps + ", lacc: " + lnxAccMax_mps + ", wacc: " + wristAccMax_degps;
        }

    }

    private static class EETimeSegments {
        public final double t_acc_s;
        public final double t_vel_s;
        public final double t_dec_s;

        public EETimeSegments(double t_acc_s, double t_vel_s, double t_dec_s) {
            this.t_acc_s = t_acc_s;
            this.t_vel_s = t_vel_s;
            this.t_dec_s = t_dec_s;
        }
    }

    private int limitingMemberIndex = 0;
    private int nonlimitingMemberIndex1 = 0;
    private int nonlimitingMemberIndex2 = 0;
    private int[] dir;
    private EETimeSegments[] timeSegments;

    private EEConstraints constraints;
    private EEState start;
    private EEState end;

    private Scurvy[] profiles;
    private double intakeRollerSpeed;
    public double totalTime_s;
    public double timeOffset_s;

    public EEProfile(EEConstraints constraints, EEState start, EEState end, double timeOffset_s,
            double intakeRollerSpeed) {
        this.start = start;
        this.end = end;
        this.constraints = constraints;
        this.timeOffset_s = timeOffset_s;
        this.intakeRollerSpeed = intakeRollerSpeed;

        profiles = new Scurvy[3];
        profiles[0] = new Scurvy(
                new Constraints(constraints.escVelMax_mps, constraints.escAccMax_mps, constraints.escJrkMax_mps),
                new Scurvy.State(start.escPos_m, start.escVel_mps),
                new Scurvy.State(end.escPos_m, end.escVel_mps));
        profiles[1] = new Scurvy(
                new Constraints(constraints.lnxVelMax_mps, constraints.lnxAccMax_mps, constraints.lnxJrkMax_mps),
                new Scurvy.State(start.lnxPos_m, start.lnxVel_mps),
                new Scurvy.State(end.lnxPos_m, end.lnxVel_mps));
        profiles[2] = new Scurvy(
                new Constraints(constraints.wristVelMax_degps, constraints.wristAccMax_degps,
                        constraints.wristJrkMax_degps),
                new Scurvy.State(start.wristPos_deg, start.wristVel_degps),
                new Scurvy.State(end.wristPos_deg, end.wristVel_degps));

        double et = profiles[0].getTotalTime();
        double lt = profiles[1].getTotalTime();
        double wt = profiles[2].getTotalTime();

        totalTime_s = timeOffset_s + Math.max(et,
                Math.max(lt, wt));

        // System.out.println("start: " + start);
        // System.out.println("end: " + end);

    }

    public double getTotalTime() {
        return totalTime_s;
    }

    public double getRollerSpeed() {
        return intakeRollerSpeed;
    }

    public EEState calculate(double time) {
        Scurvy.State esc = profiles[0].calculate(time);
        Scurvy.State lnx = profiles[1].calculate(time);
        Scurvy.State wrist = profiles[2].calculate(time);
        return new EEState(esc.p, lnx.p, wrist.p, esc.v, lnx.v, wrist.v, esc.a, lnx.a, wrist.a);
    }

}
