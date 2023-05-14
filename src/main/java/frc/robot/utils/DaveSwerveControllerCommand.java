// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.DaveSegment.Type;

/** Add your docs here. */
public class DaveSwerveControllerCommand extends CommandBase {
    private final Timer timer = new Timer();
    private final DaveSegment seg;
    private final Swerve swerve;
    public boolean danger;
    private Debouncer dangerBouncer;

    public DaveSwerveControllerCommand(DaveSegment seg) {
        this.seg = seg;
        swerve = Swerve.getInstance();
        danger = false;
        dangerBouncer = new Debouncer(0.15);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        switch (seg.type) {
            case WAIT:
                break;
            case HOLONOMIC:
                swerve.setCurrentState(swerve.AUTO);
                swerve.setTargetHeading(seg.finalHeading());
                break;
            case TRACK_TARGET_POINT:
                swerve.setCurrentState(swerve.AUTO_TARGET_POINT);
                swerve.setTargetPoint(seg.getTargetPoint());
                break;
            case LL:
                swerve.setCurrentState(swerve.AUTO_LL);
                swerve.setTargetHeading(Math.PI);
                break;
            case BALANCING:
                swerve.setCurrentState(swerve.BALANCING);
                break;
            default:
                break;
        }

        swerve.con.setThetaContraints(seg.getMaxAngularVel(), seg.getMaxAngularAcc());
        swerve.con.resetThetaCon(swerve.getPose());
    }

    public static final double DANGER_DIST = 1.0; // Distance in meters

    @Override
    public void execute() {
        switch (seg.type) {
            case BALANCING:
                break;
            default:
                double curTime = timer.get();
                var desiredState = seg.getTraj().sample(curTime);
                // Note: Safety, remove or debug and fix
                double dx2 = MathUtils.sq(desiredState.poseMeters.getX() - swerve.getPose().getX());
                double dy2 = MathUtils.sq(desiredState.poseMeters.getY() - swerve.getPose().getY());
                if (dangerBouncer.calculate(dx2 + dy2 > DANGER_DIST * DANGER_DIST)) {
                    danger = true;
                }
                // end safety

                swerve.setTargetConState(desiredState);
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        switch (seg.type) {
            case BALANCING:
                return false; // return swerve.con.isNaiveBalanaced();
            default:
                return timer.hasElapsed(seg.getTraj().getTotalTimeSeconds());
        }
    }

}
