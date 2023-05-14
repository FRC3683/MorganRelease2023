// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.function.Consumer;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.config.Constants;

/** Add your docs here. */
public class DavePigeon extends Pigeon2 {

    private boolean reversed;

    private void set(ErrorCode errorCode) {
        if (errorCode.value != 0) {
            System.out.println("----------Dead Pigeon DEAD PIGEON-------------");
            System.out.println(errorCode.name());
        }
    }

    private double prevHeadingDeg = 0;
    private double currHeadingDeg = 0;

    public DavePigeon(int canID, boolean reversed, double mountPoseYaw) {
        super(canID);
        this.reversed = reversed;

        set(configFactoryDefault());
        set(clearStickyFaults());
        set(configMountPose(mountPoseYaw, 0, 0));
        set(setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, Primes.next()));
        set(setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 10));
        set(setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 10));
        set(setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_11_GyroAccum, 20));
        set(setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, Primes.next()));
        set(setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, Primes.next()));
        set(setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, 20));
        set(setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, Primes.next()));
        set(setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, Primes.next()));
        set(resetHeading());
    }

    public ErrorCode resetHeading() {
        return setYaw(0);
    }

    public ErrorCode resetHeading(double deg) {
        return setYaw(deg);
    }

    public double getPitch() {
        return super.getPitch();
    }

    public double getRoll() {
        return super.getRoll();
    }

    public double getHeadingDeg() {
        return Math.IEEEremainder((getYaw()) * (reversed ? -1 : 1), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeadingDeg());
    }

    public void periodic() {
        prevHeadingDeg = currHeadingDeg;
        currHeadingDeg = getHeadingDeg();
    }

    public double scuffedVelocityRadSec() {
        // SmartDashboard.putNumber("pig curr", currHeadingDeg);
        // SmartDashboard.putNumber("pig prev", prevHeadingDeg);
        if(currHeadingDeg > 150 && prevHeadingDeg < -150) {
            prevHeadingDeg += 360;
        }
        if(currHeadingDeg < -150 && prevHeadingDeg > 150) {
            currHeadingDeg += 360;
        }
        return Math.toRadians(currHeadingDeg - prevHeadingDeg) / Constants.controlDt_s;
    }
}
