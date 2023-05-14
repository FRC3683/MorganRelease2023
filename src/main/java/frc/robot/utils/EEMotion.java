// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.EEProfile.EEConstraints;

/** Add your docs here. */
public class EEMotion {

    private String name;
    private EEConstraints constraints;

    private ArrayList<EEState> estates;
    private ArrayList<Double> rollerSpeeds;
    public ArrayList<EEProfile> profiles;
    private Timer timer;
    private boolean generated;
    private boolean snapshotMode;
    private double driveAwayTime_s;

    private double totalTime_s;

    public EEMotion(String name, EEConstraints constraints) {
        this.name = name;
        this.constraints = constraints;

        estates = new ArrayList<>();
        profiles = new ArrayList<>();
        rollerSpeeds = new ArrayList<>();
        timer = new Timer();
        totalTime_s = 0.0;
        generated = false;
        snapshotMode = false;
        driveAwayTime_s = -1;
    }

    public double totalTime() {
        return totalTime_s;
    }
    public double time() {
        return timer.get();
    }


    public String name() {
        return name;
    }

    public boolean canDriveAway() {
        return driveAwayTime_s == -1 ? isFinished() : timer.get() > driveAwayTime_s;
    }

    public EEMotion setSnapshotMode(boolean snapshotMode) {
        this.snapshotMode = snapshotMode;
        this.generated = false;
        return this;
    }

    public EEMotion addEEState(double rollerSpeed, double escPos_m, double lnxPos_m, double wristPos_deg,
            double escVel_mps,
            double lnxVel_mps, double wristVel_degps, double escAcc_mpsps, double lnxAcc_mpsps,
            double wristAcc_degpsps) {
        generated = false;
        rollerSpeeds.add(rollerSpeed);
        estates.add(new EEState(escPos_m, lnxPos_m, wristPos_deg, escVel_mps, lnxVel_mps, wristVel_degps, escAcc_mpsps,
                lnxAcc_mpsps, wristAcc_degpsps));
        return this;
    }

    public EEMotion addEEState(double escPos_m, double lnxPos_m, double wristPos_deg, double escVel_mps,
            double lnxVel_mps, double wristVel_degps, double escAcc_mpsps, double lnxAcc_mpsps,
            double wristAcc_degpsps) {
        return addEEState(0.0, escPos_m, lnxPos_m, wristPos_deg, escVel_mps, lnxVel_mps, wristVel_degps, escAcc_mpsps,
                lnxAcc_mpsps, wristAcc_degpsps);
    }

    public EEMotion addEEState(double escPos_m, double lnxPos_m, double wristPos_deg, double escVel_mps,
            double lnxVel_mps, double wristVel_degps) {
        return addEEState(0.0, escPos_m, lnxPos_m, wristPos_deg, escVel_mps, lnxVel_mps, wristVel_degps, 0, 0, 0);
    }

    public EEMotion addEEState(double escPos_m, double lnxPos_m, double wristPos_deg) {
        return addEEState(0.0, escPos_m, lnxPos_m, wristPos_deg, 0, 0, 0);
    }

    public EEMotion addEEState(double rollerSpeed, double escPos_m, double lnxPos_m, double wristPos_deg,
            double escVel_mps,
            double lnxVel_mps, double wristVel_degps) {
        return addEEState(rollerSpeed, escPos_m, lnxPos_m, wristPos_deg, escVel_mps, lnxVel_mps, wristVel_degps, 0, 0,
                0);
    }

    public EEMotion addEEState(double rollerSpeed, double escPos_m, double lnxPos_m, double wristPos_deg) {
        return addEEState(rollerSpeed, escPos_m, lnxPos_m, wristPos_deg, 0, 0, 0);
    }

    public EEMotion addEEState(double rollerSpeed, EEState state) {
        generated = false;
        rollerSpeeds.add(rollerSpeed);
        estates.add(state);
        return this;
    }

    public EEMotion addEEState(EEState state) {
        return addEEState(0.0, state);
    }

    public EEMotion setDriveAwayPoint() {
        driveAwayTime_s = totalTime_s;
        return this;
    }

    public EEMotion generate() {
        totalTime_s = 0.0;
        for (int i = 0; i < estates.size() - 1; i++) {
            EEProfile p;
            if (snapshotMode) {
                EEState start = new EEState(estates.get(i).escPos_m, estates.get(i).lnxPos_m,
                        estates.get(i).wristPos_deg);
                EEState end = new EEState(estates.get(i + 1).escPos_m, estates.get(i + 1).lnxPos_m,
                        estates.get(i + 1).wristPos_deg);
                p = new EEProfile(constraints, start, end, totalTime_s,
                        rollerSpeeds.get(i));
            } else {
                p = new EEProfile(constraints, estates.get(i), estates.get(i + 1), totalTime_s,
                        rollerSpeeds.get(i));
            }
            totalTime_s = p.getTotalTime();
            profiles.add(p);
        }
        generated = true;
        return this;
    }

    public void start() {
        timer.start();
    }

    public void reset() {
        timer.stop();
        timer.reset();
    }

    public boolean isFinished() {
        return timer.get() >= totalTime_s; // TODO: make better than just time check
    }

    public int getCurrentIndex() {
        if (!generated)
            return -1;
        double t = timer.get();
        int i = 0;
        if (t >= totalTime_s) {
            i = profiles.size() - 1;
        } else {
            while (t > profiles.get(i).totalTime_s) {
                i++;
            }
        }
        return i;
    }

    public EEState calculate() {
        int i = getCurrentIndex();
        if (i == -1)
            return null;
        double t = timer.get();
        double at = i == 0 ? t : t - profiles.get(i - 1).totalTime_s;
        EEState res = profiles.get(i).calculate(at);
        res.snapshot = i;
        // System.out.println(res);
        return res;
    }

    public double getRollerSpeed() {
        int i = getCurrentIndex();
        if (i == -1)
            return 0;

        return profiles.get(i).getRollerSpeed();
    }

    public EEState finalState() {
        return profiles.get(profiles.size() - 1).calculate(totalTime_s);
    }

}
