// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.DoubleStream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.Config;
import frc.robot.subsystems.EEscalator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.EEscalator.IntakePos;
import frc.robot.utils.DaveSegment.Type;
import frc.robot.utils.GlobalState.GamePiece;

/** Add your docs here. */
public class DavePath extends DaveCommandGroup {

    protected Swerve swerve;
    protected Intake intake;
    protected EEscalator escalator;
    protected DavePigeon pig;

    Timer timer;
    OI oi;
    private ArrayList<DaveSegment> segs;
    protected boolean loaded = true;
    private DavePoint initialPoint;
    private DavePoint finalPoint;
    private boolean resetOdom;

    public DavePath(boolean resetOdom) {
        this.resetOdom = resetOdom;
        swerve = Swerve.getInstance();
        escalator = EEscalator.getInstance();
        intake = Intake.getInstance();
        pig = Config.getInstance().pigeon;

        addRequirements(swerve, escalator, intake);
        oi = OI.getInstance();
        segs = new ArrayList<>();
        timer = new Timer();
    }

    public void onStart() {
    } // for subclasses to override

    @Override
    public final void initialize() {
        super.initialize();
        swerve.setCurrentState(swerve.AUTO);
        if (resetOdom) {
            if (initialPoint == null) {
                // System.err.println(segs.size());
                swerve.resetOdometry(DaveSegment.defaultPose);
            } else {
                // swerve.resetOdometry(new Pose2d(initialPoint.x, initialPoint.y,
                // Rotation2d.fromDegrees(initialPoint.head)));
                swerve.resetOdometry(new Pose2d(initialPoint.x, initialPoint.y,
                        Rotation2d.fromDegrees(initialPoint.head)));
            }
        }
        swerve.con.resetThetaCon(swerve.getPose());
        timer.start();
        onStart();
    }

    public void onExecute() {
    } // for subclasses to override

    @Override
    public final void execute() {
        super.execute();
        onExecute();
    }

    public DavePath addHolonomicSegment(double maxVel, double maxAcc, double maxAngVel, double maxAngAcc,
            double vel0_mps, double vel1_mps, double head0_deg, double head1_deg, double... points) throws Exception {
        DaveSegment s = new DaveSegment(maxVel, maxAcc, maxAngVel, maxAngAcc, vel0_mps, vel1_mps, head0_deg,
                head1_deg, false, getTotalTime(), points);
        segs.add(s);
        addCommands(s.command());
        if (initialPoint == null) {
            initialPoint = s.initialPoint();
        }
        finalPoint = s.finalPoint();
        return this;
    }

    public DavePath addStrafeSegment(double maxVel, double maxAcc, double maxAngVel, double maxAngAcc,
            double vel0_mps, double vel1_mps, double head0_deg, double head1_deg, double... points) throws Exception {
        DaveSegment s = new DaveSegment(maxVel, maxAcc, maxAngVel, maxAngAcc, vel0_mps, vel1_mps, head0_deg,
                head1_deg, true, getTotalTime(), points);
        segs.add(s);
        addCommands(s.command());
        if (initialPoint == null) {
            initialPoint = s.initialPoint();
        }
        finalPoint = s.finalPoint();
        return this;
    }

    public DavePath addBalanceSegment() {
        DaveSegment bal = new DaveSegment();
        segs.add(bal);
        addCommands(bal.command());
        return this;
    }

    public DavePath appendHolonomicSegment(double maxVel, double maxAcc, double maxAngVel, double maxAngAcc,
            double vel_mps, double head_deg, double... points) throws Exception {
        if (finalPoint == null) {
            throw new Exception(
                    "Append to what? Look at yourself, trying to attach endpoints to segments that don't exist. Maybe go attach some meaning to your life.");
        }
        double[] newpoints = DoubleStream.concat(Arrays.stream(finalPoint.segpoint), Arrays.stream(points)).toArray();
        DaveSegment s = new DaveSegment(maxVel, maxAcc, maxAngVel, maxAngAcc, finalPoint.vel, vel_mps,
                finalPoint.head, head_deg, false, getTotalTime(), newpoints);
        segs.add(s);
        addCommands(s.command());
        finalPoint = s.finalPoint();
        return this;
    }

    public DavePath appendStrafeSegment(double maxVel, double maxAcc, double maxAngVel, double maxAngAcc,
            double vel_mps, double head_deg, double... points) throws Exception {
        if (finalPoint == null) {
            throw new Exception(
                    "Append to what? Look at yourself, trying to attach endpoints to segments that don't exist. Maybe go attach some meaning to your life.");
        }
        double[] newpoints = DoubleStream.concat(Arrays.stream(finalPoint.segpoint), Arrays.stream(points)).toArray();
        DaveSegment s = new DaveSegment(maxVel, maxAcc, maxAngVel, maxAngAcc, finalPoint.vel, vel_mps,
                finalPoint.head, head_deg, true, getTotalTime(), newpoints);
        segs.add(s);
        addCommands(s.command());
        finalPoint = s.finalPoint();
        return this;
    }

    public DavePath addPathWeaverSegment(double maxVel, double maxAcc, double maxAngVel, double maxAngAcc,
            double vel0_mps, double vel1_mps, double head0_deg, double head1_deg, String traj) throws Exception {
        DaveSegment s = new DaveSegment(maxVel, maxAcc, maxAngVel, maxAngAcc, vel0_mps, vel1_mps, head0_deg,
                head1_deg, false, getTotalTime(), traj);
        segs.add(s);
        addCommands(s.command());
        if (initialPoint == null) {
            initialPoint = s.initialPoint();
        }
        finalPoint = s.finalPoint();
        return this;
    }

    public DavePath addTargetTrackingSegment(double maxVel, double maxAcc, double maxAngVel, double maxAngAcc,
            double vel0_mps, double vel1_mps, double head0_deg, double targx, double targy, double... points)
            throws Exception {
        DaveSegment s = new DaveSegment(maxVel, maxAcc, maxAngVel, maxAngAcc, vel0_mps, vel1_mps, head0_deg,
                new DavePoint(targx, targy), getTotalTime(), points);
        segs.add(s);
        addCommands(s.command());
        if (initialPoint == null) {
            initialPoint = s.initialPoint();
        }
        finalPoint = s.finalPoint();
        return this;
    }

    public DavePath appendTargetTrackingSegment(double maxVel, double maxAcc, double maxAngVel,
            double maxAngAcc, double vel_mps, double targx, double targy, double... points) throws Exception {
        if (finalPoint == null) {
            throw new Exception(
                    "Append to what? Look at yourself, trying to attach endpoints to segments that don't exist. Maybe go attach some meaning to your life.");
        }
        double[] newpoints = DoubleStream.concat(Arrays.stream(finalPoint.segpoint), Arrays.stream(points)).toArray();
        DaveSegment s = new DaveSegment(maxVel, maxAcc, maxAngVel, maxAngAcc, finalPoint.vel, vel_mps,
                finalPoint.head, new DavePoint(targx, targy), getTotalTime(), newpoints);
        segs.add(s);
        addCommands(s.command());
        finalPoint = s.finalPoint();
        return this;
    }

    public DavePath addTargetTrackingSegment(double maxVel, double maxAcc, double maxAngularVel, double maxAngularAcc,
            double vel0_mps, double vel1_mps, double head0_deg, double targx, double targy, String traj)
            throws Exception {
        DaveSegment s = new DaveSegment(maxVel, maxAcc, maxAngularVel, maxAngularAcc, vel0_mps, vel1_mps, head0_deg,
                new DavePoint(targx, targy), getTotalTime(), traj);
        segs.add(s);
        addCommands(s.command());
        if (initialPoint == null) {
            initialPoint = s.initialPoint();
        }
        finalPoint = s.finalPoint();
        return this;
    }

    public DavePath addWaitSegment(double time) {
        DaveSegment s = new DaveSegment(time, getTotalTime());
        segs.add(s);
        addCommands(s.command());
        return this;
    }

    public double getTotalTime() {
        if (segs.isEmpty())
            return 0.0;
        return segs.get(segs.size() - 1).getTotalTime();
    }

    public double time() {
        return timer.get();
    }

    public void reset() {
        timer.reset();
    }

    public boolean after(double seconds) {
        return timer.hasElapsed(seconds);
    }

    public boolean before(double seconds) {
        return !timer.hasElapsed(seconds);
    }

    public boolean between(double earlier, double later) {
        return after(earlier) && before(later);
    }

    public double alpha(double earlier, double later) {
        double t = timer.get();
        if (t <= earlier)
            return 0.0;
        if (t >= later)
            return 1.0;
        return MathUtils.unlerp(earlier, later, t);
    }

    protected void trackHolonomic() {
        swerve.setCurrentState(swerve.AUTO);
    }

    protected void trackTargetPoint() {
        swerve.setCurrentState(swerve.AUTO_TARGET_POINT);
    }

    protected double segEnd(int i) {
        return segs.get(i).getTotalTime();
    }

    protected int getSeg() {
        int i = segs.size() - 1;
        double t = timer.get();
        if (t >= segs.get(segs.size() - 1).getTotalTime()) {
            i = segs.size() - 1;
        } else {
            while (t > segs.get(i).getTotalTime()) {
                i++;
            }
        }
        return i;
    }

    @Override
    public boolean isFinished() {
        boolean safety = false;
        DaveSegment seg = segs.get(getSeg());
        if (seg.type == Type.HOLONOMIC || seg.type == Type.LL || seg.type == Type.TRACK_TARGET_POINT) {
            safety = ((DaveSwerveControllerCommand) seg.command()).danger;
        }
        return safety || super.isFinished(); // NOTE: remove safety to disable safety
    }

    protected void emote(EEMotion motion) {
        escalator.setEmotion(motion, escalator.ZEROING);
        intake.setEEMotion(motion);
        intake.setCurrentState(intake.EEMOTIONAL);
    }

    protected void intakeCube() {
        GlobalState.getInstance().gamepiece = GamePiece.CUBE;
        escalator.intakePos = IntakePos.CUBE_AUTO;
        intake.setCurrentState(intake.INTAKECUBE);
        escalator.setCurrentState(escalator.INTAKING);
    }

    protected void stow() {
        if (escalator.STOWED.isActive() || escalator.EEMOTIONAL.isActive()) {
            return;
        }
        escalator.setCurrentState(escalator.STOWING);
        intake.setCurrentState(intake.HOLD);
    }

    protected void cubeLLNear() {
        LL.TOP.LEDon();
        LL.TOP.setPipeline(2);
    }

    protected void cubeLLFar() {
        LL.TOP.LEDon();
        LL.TOP.setPipeline(4);
    }

    public void onComplete(boolean interrupted) {
    } // for subclasses to override

    @Override
    public final void end(boolean interuppted) {
        // oi.rumbleDriverFor(0.2, 0.2, 2.0);
        swerve.setCurrentState(swerve.DISABLED);
        super.end(interuppted);
        onComplete(interuppted);
    }

}
