// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.net.ConnectException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.AutoConstants;
import frc.robot.config.Config;
import frc.robot.config.Constants;
import frc.robot.utils.GlobalState.GamePiece;
import frc.robot.utils.GlobalState.ScoringLevel;
import frc.robot.utils.MultiStageController.ControllerCondition;

/** Add your docs here. */
public class DaveSwerveController {

    public static final State defaultState = new State(new Pose2d(), 0, 0);

    public static class State {
        public Pose2d pose;
        public double vel_m_s;
        public double acc_m_s2;

        public State(Pose2d pose, double vel_m_s, double acc_m_s2) {
            this.pose = pose;
            this.vel_m_s = vel_m_s;
            this.acc_m_s2 = acc_m_s2;
        }
    }

    private Pose2d poseError;
    private Rotation2d rotationError;
    private Pose2d poseTolerance;
    private boolean enabled;
    private boolean firstRun;

    private final Debouncer readyToScoreDebounce;
    private final Debouncer readyToShootDebounce;
    private final PIDController xCon;
    private final PIDController yCon;
    private final ProfiledPIDController thetaCon;
    private final PIDController headCon;
    private final PIDController llCon;
    private final TwoStagePID ll_strafe;
    private final TwoStagePID ll_far_strafe;
    private final TwoStagePID snap180;
    private final TwoStagePID ll_dist;
    private final double xTiltP;
    private final double yTiltP;
    private final MultiStageController pitchCon;
    private final MultiStageController rollCon;
    private final TwoStagePID balance_pid;
    private final double balance_deadzone_degs_x;
    private final double balance_done_degs;

    private final DavePigeon pig;

    GlobalState globalState;

    public DaveSwerveController(double xP, double xI, double xD, double yP, double yI, double yD, double thetaP,
            double thetaI, double thetaD, double headP, double headI, double headD, double llP, double llI, double llD,
            double xTiltP, double yTiltP,
            double balanceP, double balanceI, double balanceD) {

        xCon = new PIDController(xP, xI, xD, Constants.controlDt_s);
        yCon = new PIDController(yP, yI, yD, Constants.controlDt_s);
        thetaCon = new ProfiledPIDController(thetaP, thetaI, thetaD, AutoConstants.kThetaControllerConstraints,
                +Constants.controlDt_s);
        // thetaCon.enableContinuousInput(-180, 180);
        thetaCon.enableContinuousInput(-Math.PI, Math.PI);
        headCon = new PIDController(headP, headI, headD, Constants.controlDt_s);
        headCon.enableContinuousInput(-Math.PI, Math.PI);
        llCon = new PIDController(llP, llI, llD, Constants.controlDt_s);
        ll_strafe = new TwoStagePID(new PIDController(-3.8, 0, 0.0),
                new PIDController(-4.0, 0.0, 0.0, Constants.controlDt_s), 0.085, 0.0025, 0.0075);
        ll_strafe.enableMinMaxOutput(-Constants.swerveTrueMaxSpeed_mps, Constants.swerveTrueMaxSpeed_mps);
        ll_far_strafe = new TwoStagePID(new PIDController(0.2, 0, 0.0),
                new PIDController(0.0, 0.0, 0.0, Constants.controlDt_s), 0, 0.0, 0.0);
        ll_far_strafe.enableMinMaxOutput(-Constants.swerveTrueMaxSpeed_mps, Constants.swerveTrueMaxSpeed_mps);
        snap180 = new TwoStagePID(new PIDController(5.1, 0.0, 0.1, Constants.controlDt_s),
                new PIDController(14.6, 0.03, 0.02, Constants.controlDt_s), 0.15, 0.01, 0.02);// Math.PI*0.04, 0.01,
                                                                                              // 0.02);
        snap180.enableContinuousOutput(-Math.PI, Math.PI);
        snap180.enableMinMaxOutput(-Constants.swerveMaxTurn_radps, Constants.swerveMaxTurn_radps);
        ll_dist = new TwoStagePID(new PIDController(2.3, 0, 0.01, Constants.controlDt_s),
                new PIDController(0.0, 0, 0.01, Constants.controlDt_s), 0, 0.005, 0.02);
        ll_dist.enableMinMaxOutput(-Constants.swerveTrueMaxSpeed_mps, -0.15);
        this.xTiltP = xTiltP;
        this.yTiltP = yTiltP;
        readyToScoreDebounce = new Debouncer(0.15);
        readyToShootDebounce = new Debouncer(0.05);

        pitchCon = new MultiStageController(Constants.controlDt_s);
        pitchCon.addCon(balanceP, balanceI, balanceD);
        rollCon = new MultiStageController(Constants.controlDt_s);
        rollCon.addCon(balanceP, balanceI, balanceD);
        pitchCon.setSetpoint(0.0);
        rollCon.setSetpoint(0.0);

        balance_pid = new TwoStagePID(new PIDController(0.065, 0, 0.0), new PIDController(0.03, 0, 0), 12.3, 0, 0);
        balance_pid.enableMinMaxOutput(-0.8, 0.8);
        // balance_pid.debounceThreshhold(0.03);
        balance_deadzone_degs_x = 7;
        balance_done_degs = 2;

        poseError = new Pose2d();
        rotationError = new Rotation2d();
        poseTolerance = new Pose2d();
        enabled = true;
        firstRun = true;

        globalState = GlobalState.getInstance();

        pig = Config.getInstance().pigeon;
    }

    public void resetThetaCon(Pose2d currentPose) {
        thetaCon.reset(currentPose.getRotation().getRadians());
    }

    public boolean atReference() {
        final var eTranslate = poseError.getTranslation();
        final var eRotate = rotationError;
        final var tolTranslate = poseTolerance.getTranslation();
        final var tolRotate = poseTolerance.getRotation();
        return Math.abs(eTranslate.getX()) <= tolTranslate.getX()
                && Math.abs(eTranslate.getY()) <= tolTranslate.getY()
                && Math.abs(eRotate.getRadians()) <= tolRotate.getRadians();
    }

    public void setTolerance(double xtol, double ytol, double thetaTol_deg) {
        poseTolerance = new Pose2d(xtol, ytol, Rotation2d.fromDegrees(thetaTol_deg));
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void setThetaContraints(double max_radps, double max_radps2) {
        thetaCon.setConstraints(new Constraints(max_radps, max_radps2));
    }

    public void setDefaultThetaContraints() {
        thetaCon.setConstraints(AutoConstants.kThetaControllerConstraints);
    }

    public PIDController getXCon() {
        return xCon;
    }

    public PIDController getYCon() {
        return yCon;
    }

    public ProfiledPIDController getThetaCon() {
        return thetaCon;
    }

    public double headingCorrection(double lastHeading, double currentHeading) {
        return headCon.calculate(currentHeading, lastHeading);
    }

    public ChassisSpeeds calculate(Pose2d currentPose, ChassisSpeeds s, boolean robotRelative) {
        // If this is the first run, then we need to reset the theta controller to the
        // current pose's
        // heading.
        if (firstRun) {
            thetaCon.reset(currentPose.getRotation().getRadians());
            firstRun = false;
        }

        // Calculate feedforward velocities (field-relative).
        double xFF = s.vxMetersPerSecond;
        double yFF = s.vyMetersPerSecond;
        double thetaFF = s.omegaRadiansPerSecond;

        // SmartDashboard.putNumber("target omegaRad", s.omegaRadiansPerSecond);
        // SmartDashboard.putNumber("curr omegaRad", (pig.scuffedVelocityRadSec() *
        // Math.PI / 180.0));

        return robotRelative ? new ChassisSpeeds(xFF, yFF, thetaFF)
                : ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, thetaFF, currentPose.getRotation());
    }

    private double snap180calculate(double curr, double target) {
        return snap180.calculate(curr, target);
    }

    public ChassisSpeeds calculate180(Pose2d currentPose, ChassisSpeeds s, double rads) {

        // Calculate feedforward velocities (field-relative).
        double xFF = s.vxMetersPerSecond;
        double yFF = s.vyMetersPerSecond;

        // SmartDashboard.putNumber("target omegaRad", s.omegaRadiansPerSecond);
        // SmartDashboard.putNumber("curr omegaRad", (pig.scuffedVelocityRadSec() *
        // Math.PI / 180.0));

        return ChassisSpeeds
                .fromFieldRelativeSpeeds(xFF, yFF,
                        MathUtils.clamp(-Constants.swerveMaxTurn_radps, Constants.swerveMaxTurn_radps,
                                snap180calculate(currentPose.getRotation().getRadians(), rads)),
                        currentPose.getRotation());
    }

    public ChassisSpeeds calculate(Pose2d currentPose, State desiredState,
            Rotation2d angleRef) {
        // If this is the first run, then we need to reset the theta controller to the
        // current pose's
        // heading.
        if (firstRun) {
            thetaCon.reset(currentPose.getRotation().getRadians());
            firstRun = false;
        }

        // Calculate feedforward velocities (field-relative).
        double xFF = desiredState.vel_m_s * desiredState.pose.getRotation().getCos();
        double yFF = desiredState.vel_m_s * desiredState.pose.getRotation().getSin();
        double thetaFF = thetaCon.calculate(currentPose.getRotation().getRadians(), angleRef.getRadians());

        poseError = desiredState.pose.relativeTo(currentPose);
        rotationError = angleRef.minus(currentPose.getRotation());

        if (!enabled) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, thetaFF, currentPose.getRotation());
        }

        // Calculate feedback velocities (based on position error).
        double xFeedback = xCon.calculate(currentPose.getX(), desiredState.pose.getX());
        double yFeedback = yCon.calculate(currentPose.getY(), desiredState.pose.getY());

        // Return next output.
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xFF + xFeedback, yFF + yFeedback, thetaFF, currentPose.getRotation());
    }

    public ChassisSpeeds calculate(Pose2d currentPose, double x_vel, double y_vel, double theta_rad) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                x_vel, y_vel, thetaCon.calculate(currentPose.getRotation().getRadians(), theta_rad),
                currentPose.getRotation());
    }

    Rotation2d scoringHeading = Rotation2d.fromRadians(Math.PI);

    public ChassisSpeeds calculateLL(Pose2d currentPose, State desiredState, boolean found_target, double targetY) {
        // Calculate feedforward velocities (field-relative).
        double deltaRad = scoringHeading.getRadians() - currentPose.getRotation().getRadians();
        double xFF = desiredState.vel_m_s * desiredState.pose.getRotation().getCos();
        double yFF = desiredState.vel_m_s * desiredState.pose.getRotation().getSin();
        double thetaFF = thetaCon.calculate(currentPose.getRotation().getRadians(), scoringHeading.getRadians());

        poseError = desiredState.pose.relativeTo(currentPose);
        rotationError = Rotation2d.fromRadians(deltaRad);

        // Calculate feedback velocities (based on position error).
        double xFeedback = xCon.calculate(currentPose.getX(), desiredState.pose.getX());
        double yFeedback = 0;
        if (Math.abs(deltaRad) < 0.07 && LL.TOP.validTarget()) {
            yFeedback = ll_strafe.calculate(distanceLeftRight(), 0);
            // System.out.println("bing");
        } else if (found_target && Math.abs(deltaRad) < 0.07) {
            yFeedback = yCon.calculate(currentPose.getY(), targetY);
            // System.out.println("chilen");
        } else {
            yFeedback = yCon.calculate(currentPose.getY(), desiredState.pose.getY());

        }

        // Return next output.
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xFF + xFeedback, yFF + yFeedback, thetaFF, currentPose.getRotation());
    }

    public double distance() {
        final double height_ll = 1.012;
        final double height_mid = 0.611188;
        final double height_420 = 1.1144;
        final double height_tag = 0.46;
        double heightDiff = 0;
        if (globalState.gamepiece == GamePiece.CONE) {
            heightDiff = (globalState.scoringLevel == ScoringLevel.HIGH ? height_420 : height_mid) - height_ll;
        } else {
            heightDiff = height_ll - height_tag;
        }
        return Math.abs(heightDiff / Math.tan(Math.toRadians(LL.TOP.tx() + 19)));
    }

    public double distanceCS() {
        final double height_ll = 1.012 + 0.2286;
        final double height_mid = 0.611188;
        final double height_420 = 1.1144;
        final double height_tag = 0.46;
        double heightDiff = 0;
        heightDiff = height_ll - height_tag;
        return Math.abs(heightDiff / Math.tan(Math.toRadians(LL.TOP.tx() + 19)));
    }

    private double ty() {
        return LL.TOP.ty() + 0.82;
    }

    private double distanceLeftRight() {
        return Math.tan(Math.toRadians(ty())) * distance();
    }

    private boolean thetaClose(Pose2d currentPose) {
        return Math.abs(currentPose.getRotation().getRadians()) > Math.PI - 0.07;
    }

    private boolean thetaReadyToScore(Pose2d currentPose) {
        return Math.abs(currentPose.getRotation().getRadians()) > Math.PI - 0.1;

        // return snap180.atReference();
    }

    public boolean leftRightClose() {
        return Math.abs(ty()) < 12.0;
    }

    private double distanceSetpoint() {
        if (globalState.gamepiece == GamePiece.CONE) {
            return globalState.scoringLevel == ScoringLevel.HIGH ? 0.95 : 0.55;
        }
        return 0.47;
    }

    private boolean distReadyToScore() {
        double fudge = 0.16; // cube
        if (globalState.gamepiece == GamePiece.CONE) {// } && globalState.scoringLevel == ScoringLevel.HIGH) {
            fudge = globalState.scoringLevel == ScoringLevel.HIGH ? 0.31 : 0.22;
        }
        return LL.TOP.validTarget() && distance() < distanceSetpoint() + fudge;
    }

    private boolean distReadyToShoot() {
        double fudge = 5.8; // cube
        return LL.TOP.validTarget() && distance() < fudge;
    }

    private double leftRightSetpoint() {
        return globalState.scoringOffset.offset(globalState.gamepiece);
    }

    public boolean leftRightReady() {
        double tolerance = 0.02; // CONE
        if (globalState.gamepiece == GamePiece.CUBE) {
            tolerance = 0.05;
        }
        return LL.TOP.validTarget() && Math.abs(distanceLeftRight() - leftRightSetpoint()) < tolerance;
    }

    public boolean leftRightStrafeReady() {
        return LL.TOP.validTarget() && Math.abs(LL.TOP.ty()) < 1;
    }

    public boolean readyToScore(Pose2d currentPose) {
        return readyToScoreDebounce.calculate(thetaReadyToScore(currentPose) && leftRightReady() && distReadyToScore());
    }

    public boolean readyToShoot(Pose2d currentPose) {
        return readyToShootDebounce.calculate(thetaReadyToScore(currentPose) && leftRightReady() && distReadyToShoot());
    }

    public ChassisSpeeds calculateLLClose(Pose2d currentPose) {
        // SmartDashboard.putNumber("dist setpoint", distanceSetpoint());
        // SmartDashboard.putNumber("dist", distance());
        // SmartDashboard.putNumber("left right", distanceLeftRight());
        // SmartDashboard.putBoolean("dist ready", distReadyToScore());
        // SmartDashboard.putBoolean("left right ready", leftRightReady());

        double thing = -1.0;
        if (DriverStation.isAutonomous()) {
            thing = -1.5;
        }

        double dist_calc = MathUtils.clamp(-2.0, -0.3, ll_dist.calculate(distance(), distanceSetpoint()));
        double leftRightCalc = ll_strafe.calculate(distanceLeftRight(), leftRightSetpoint()); // force pid loop to run
        // regardless of conditions,
        // makes atReference() correct
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                !LL.TOP.validTarget() || !thetaClose(currentPose) || !leftRightClose() ? thing : dist_calc,
                !LL.TOP.validTarget() || !thetaClose(currentPose) ? 0 : leftRightCalc,
                snap180calculate(currentPose.getRotation().getRadians(), Math.PI),
                currentPose.getRotation());
    }

    public ChassisSpeeds calculateLLFar(Pose2d currentPose) {
        double leftRightCalc = ll_far_strafe.calculate(LL.TOP.ty(), 0); // force pid loop to run regardless of
                                                                        // conditions, makes atReference() correct
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                0,
                !LL.TOP.validTarget() || !thetaClose(currentPose) ? 0 : leftRightCalc,
                snap180calculate(currentPose.getRotation().getRadians(), Math.PI),
                currentPose.getRotation());
    }

    public ChassisSpeeds tiltCorrection(ChassisSpeeds s, double pitch, double roll) {
        boolean yesMove = pitch * pitch + roll * roll > 1.0;
        double correctionPitch = yesMove ? xTiltP * pitch : 0;
        double correctionRoll = yesMove ? yTiltP * roll : 0;

        return new ChassisSpeeds(s.vxMetersPerSecond + correctionPitch, s.vyMetersPerSecond - correctionRoll,
                s.omegaRadiansPerSecond);
    }

    public ChassisSpeeds naiveBalance() {
        double direction = Math.abs(pig.getHeadingDeg()) > 90 ? 1 : -1;
        double roll = pig.getRoll() * direction;
        double calc = balance_pid.calculate(roll);
        return new ChassisSpeeds(calc, 0, 0);
    }

    public boolean isNaiveBalanaced() {
        return Math.abs(pig.getRoll()) < balance_done_degs;
    }

    public ChassisSpeeds balance(double pitch, double roll) {
        final double deadzone = 5;
        if (pitch + roll < deadzone) {
            return new ChassisSpeeds(0, 0,
                    0.0);
        }
        return new ChassisSpeeds(pitchCon.calculate(pitch), rollCon.calculate(roll),
                0.0);
    }

    public boolean angleClose180(Pose2d currentPose) {
        return MathUtils.closeEnough(Math.abs(currentPose.getRotation().getRadians()), Math.PI, Math.PI / 10);
    }

}
