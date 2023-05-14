package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.config.Constants;
import frc.robot.utils.DaveSwerveModuleState2;

public class SwerveModule {

    public final CANSparkMax driveMotor;
    public final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final DutyCycleEncoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    boolean manualZero = false;

    private SparkMaxPIDController drivingController;
    private SparkMaxPIDController turningController;

    private double lastVel;
    private double lastAng;

    private final double turnkV;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed,
            int driveCurrentLimit_A, int turnCurrentLimit_A) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new DutyCycleEncoder(absoluteEncoderId);
        absoluteEncoder.setDutyCycleRange(1.0 / 4096, 4095.0 / 4096);

        Timer.delay(0.1);
        lastVel = 0.0;
        lastAng = 0.0;

        // if(turningMotorId == 2) {
        // manualZero = true;
        // }

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
        if (!Config.flashHasBeenBurned) {
            driveMotor.clearFaults();
            turningMotor.clearFaults();
            driveMotor.restoreFactoryDefaults(false);
            turningMotor.restoreFactoryDefaults(false);
            driveMotor.enableVoltageCompensation(Constants.nominalVoltage);
            turningMotor.enableVoltageCompensation(Constants.nominalVoltage);
            driveMotor.setSmartCurrentLimit(driveCurrentLimit_A);
            turningMotor.setSmartCurrentLimit(turnCurrentLimit_A);
            driveMotor.setIdleMode(IdleMode.kBrake);
            turningMotor.setIdleMode(IdleMode.kBrake);

            driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
            driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
            driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);

            turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
            turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
            turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);

            Timer.delay(0.1);
            driveMotor.setInverted(driveMotorReversed);
            turningMotor.setInverted(turningMotorReversed);
            Timer.delay(0.1);
        }

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        boolean comp = Preferences.getBoolean("comp", true);
        double fudge = comp ? Constants.swerveDistFudgeComp : Constants.swerveDistFudgePractice;
        if (!Config.flashHasBeenBurned) {
            driveEncoder.setPositionConversionFactor(Constants.motorDistanceMeters * fudge);
            driveEncoder.setVelocityConversionFactor(Constants.motorDistanceMeters * fudge / 60.0); // TODO
            turningEncoder.setPositionConversionFactor(Math.PI * 2 * 7.0 / 150.0);
            turningEncoder.setVelocityConversionFactor(Math.PI * 2 * 7.0 / 150.0);
        }

        drivingController = driveMotor.getPIDController();
        drivingController.setFeedbackDevice(driveEncoder);
        turningController = turningMotor.getPIDController();
        turningController.setFeedbackDevice(turningEncoder);

        turnkV = 0.5;

        if (!Config.flashHasBeenBurned) {

            if (comp) {

                drivingController.setFF(0.206, 0);
                drivingController.setP(0.03, 0);
                drivingController.setI(0.0004, 0);
                drivingController.setD(0.0, 0);
                drivingController.setIZone(0.05, 0);
                drivingController.setOutputRange(-1, 1, 0);

                turningController.setP(1.2, 0);
                turningController.setI(0.000, 0);
                turningController.setD(0.0, 0);
                turningController.setIZone(0.0, 0);
                turningController.setOutputRange(-1, 1, 0);

                // turnkV = 0.0;
            } else {

                drivingController.setFF(0.2, 0);
                drivingController.setP(0.022, 0);
                drivingController.setI(0.0004, 0);
                drivingController.setD(0.0, 0);
                drivingController.setIZone(0.05, 0);
                drivingController.setOutputRange(-1, 1, 0);

                turningController.setP(1.2, 0);
                turningController.setI(0.000, 0);
                turningController.setD(0.0, 0);
                turningController.setIZone(0.0, 0);
                turningController.setOutputRange(-1, 1, 0);

                // turnkV = 0;

            }
        }

    }

    public void burnFlash() {
        driveMotor.burnFlash();
        turningMotor.burnFlash();
    }

    public void setSpeed(double metersPerSecond) {
        drivingController.setReference(metersPerSecond, ControlType.kVelocity, 0);
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRaw() { // revolutions
        return absoluteEncoder.getAbsolutePosition() * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public double getAbsoluteEncoderRad() {
        return getAbsoluteEncoderRaw() * Math.PI * 2 + absoluteEncoderOffsetRad;
    }

    public SwerveModulePosition odom() {
        return new SwerveModulePosition(driveEncoder.getPosition(),
                Rotation2d.fromRadians(turningEncoder.getPosition()));
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(manualZero ? 0 : getAbsoluteEncoderRad());
    }

    public void zeroEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(0);
    }

    public DaveSwerveModuleState2 getState() {
        double vel = getDriveVelocity();
        Rotation2d azimuth = Rotation2d.fromRadians(getTurningPosition());
        double omega = getTurningVelocity();
        return new DaveSwerveModuleState2(vel, azimuth, omega);
    }

    public double boundedAngle(double setpoint) {
        return turningEncoder.getPosition()
                + MathUtil.inputModulus(setpoint - turningEncoder.getPosition(), -Math.PI, Math.PI);
    }

    public void setDesiredState(DaveSwerveModuleState2 state, boolean speedCheck, boolean optimize, boolean dashboard) {
        if (speedCheck && Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        if (optimize) {
            SwerveModuleState simpleState = new SwerveModuleState(state.speedMetersPerSecond, state.angle);
            simpleState = SwerveModuleState.optimize(simpleState, getState().angle);
            state = new DaveSwerveModuleState2(
                    simpleState.speedMetersPerSecond, simpleState.angle, state.omegaRadPerSecond);
        }
        if (lastVel != state.speedMetersPerSecond) {
            drivingController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        }
        lastVel = state.speedMetersPerSecond;

        // double angOutput = turningPidController.calculate(getTurningPosition(),
        // state.angle.getRadians());
        // if (lastAng != angOutput) {
        // turningMotor.set(angOutput);
        // }
        // lastAng = angOutput;
        double angOutput = boundedAngle(state.angle.getRadians());
        if (lastAng != angOutput) {
            turningController.setReference(angOutput, ControlType.kPosition, 0, state.omegaRadPerSecond * turnkV,
                    ArbFFUnits.kVoltage);
        }
        lastAng = angOutput; // state.angle.getRadians();
        if (dashboard) {
            // SmartDashboard.putString("Swerve[" + absoluteEncoder.getSourceChannel() + "]
            // state", state.toString());
        }
    }

    public void setDesiredState(DaveSwerveModuleState2 state) {
        setDesiredState(state, true, true, true);
    }

    public void setRawPowers(double speed, double turn) {
        driveMotor.set(speed);
        turningMotor.set(turn);
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
        lastVel = 0.0;
        lastAng = 0.0;
    }
}