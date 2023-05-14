// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class EEState {
    public final double escPos_m;
    public final double lnxPos_m;
    public final double wristPos_deg;
    public final double escVel_mps;
    public final double lnxVel_mps;
    public final double wristVel_degps;
    public final double escAcc_mpsps;
    public final double lnxAcc_mpsps;
    public final double wristAcc_degpsps;

    public int snapshot;

    public EEState(double escPos_m, double lnxPos_m, double wristPos_deg, double escVel_mps, double lnxVel_mps,
            double wristVel_degps, double escAcc_mpsps, double lnxAcc_mpsps, double wristAcc_degpsps, int snapshot) {
        this.escPos_m = escPos_m;
        this.lnxPos_m = lnxPos_m;
        this.wristPos_deg = wristPos_deg;
        this.escVel_mps = escVel_mps;
        this.lnxVel_mps = lnxVel_mps;
        this.wristVel_degps = wristVel_degps;
        this.escAcc_mpsps = escAcc_mpsps;
        this.lnxAcc_mpsps = lnxAcc_mpsps;
        this.wristAcc_degpsps = wristAcc_degpsps;
        this.snapshot = snapshot;
    }

    public EEState(double escPos_m, double lnxPos_m, double wristPos_deg, double escVel_mps, double lnxVel_mps,
            double wristVel_degps, double escAcc_mpsps, double lnxAcc_mpsps, double wristAcc_degpsps) {
        this(escPos_m, lnxPos_m, wristPos_deg, escVel_mps, lnxVel_mps, wristVel_degps, escAcc_mpsps, lnxAcc_mpsps,
                wristAcc_degpsps, 0);
    }

    public EEState(double escPos_m, double lnxPos_m, double wristPos_deg, double escVel_mps, double lnxVel_mps,
            double wristVel_degps) {
        this(escPos_m, lnxPos_m, wristPos_deg, escVel_mps, lnxVel_mps, wristVel_degps, 0, 0, 0);
    }

    public EEState(double escPos_m, double lnxPos_m, double wristPos_deg) {
        this(escPos_m, lnxPos_m, wristPos_deg, 0, 0, 0, 0, 0, 0);
    }

    @Override
    public String toString() {
        return String.format("epos: %f, lpos: %f, wpos: %f",
                escPos_m, lnxPos_m, wristPos_deg);
    }
}
