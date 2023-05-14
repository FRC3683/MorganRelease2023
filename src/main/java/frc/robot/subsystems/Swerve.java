package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.AutoConstants;
import frc.robot.config.Constants;
import frc.robot.utils.AutoSelector;
import frc.robot.utils.DavePigeon;
import frc.robot.utils.DavePoint;
import frc.robot.utils.DaveSwerveController;
import frc.robot.utils.DaveSwerveKin2;
import frc.robot.utils.DaveSwerveModuleState2;
import frc.robot.utils.GlobalState;
import frc.robot.utils.LL;
import frc.robot.utils.MathUtils;
import frc.robot.utils.GlobalState.GamePiece;

public class Swerve extends DaveSubsystem {
    private static Swerve instance;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    public final State DISABLED, IDLE, ROBOT_RELATIVE, FIELD_RELATIVE_CLASSIC, FIELD_RELATIVE_ABSOLUTE, SNAP_TOWARD,
            SNAP_AWAY, SNAP_HP, AUTO,
            AUTO_TARGET_POINT, AUTO_LL, PREBALANCING_TOWARDS, PREBALANCING_AWAY, BALANCING, BASE_LOCK_PASSIVE,
            BASE_LOCK_ACTIVE, STTBOLLS, STTBOLLS_AUTO, STRAFING, SCORING;

    private final SwerveModule sm_fl, sm_fr, sm_br, sm_bl;

    private final DavePigeon pigeon;

    public final SwerveDriveKinematics kin;
    public final DaveSwerveKin2 kin2;
    private final SwerveDriveOdometry odom;
    private final SwerveDriveOdometry odomTrue;
    // private final SwerveDrivePoseEstimator pe;
    public final DaveSwerveController con;

    private final SlewRateLimiter xlim;
    private final SlewRateLimiter ylim;
    private final SlewRateLimiter thetalim;

    private Field2d field;
    // private Field2d fieldv;

    /**
     * Standard deviations of model states. Increase these numbers to trust your
     * model's state estimates less. This matrix is in the form [x, y, theta]ᵀ,
     * with units in meters and radians, then meters.
     */
    // private static final Vector<N3> STATE_STDS = VecBuilder.fill(0.05, 0.05,
    // Units.degreesToRadians(0.5));

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision less. This matrix is in the form
     * [x, y, theta]ᵀ, with units in meters and radians.
     */
    // private static final Vector<N3> VISION_STDS = VecBuilder.fill(0.1, 0.1,
    // Units.degreesToRadians(10));

    private AutoSelector as;

    private double xspeed_mps;
    private double yspeed_mps;
    private double turnspeed_radps;

    private Rotation2d holonomicTarget = new Rotation2d();
    private DavePoint targetPoint;
    private DaveSwerveController.State targetConState;

    // public boolean recording = false;
    // public ArrayList<Pose2d> recordedPath = new ArrayList<Pose2d>();

    private final GlobalState globalState;

    // Convention: X is forward positive, Y is left positive, angle is ccw positive
    public void fromController(double xspeed, double yspeed, double turnspeed) {
        this.xspeed_mps = xspeed * Constants.swerveMaxSpeed_mps; // TODO: figure out scaling factors
        this.yspeed_mps = yspeed * Constants.swerveMaxSpeed_mps;
        this.turnspeed_radps = turnspeed * Constants.swerveMaxTurn_radps;
    }

    public void fromControllerRaw(double xspeed, double yspeed, double turnspeed) {
        this.xspeed_mps = xspeed; // TODO: figure out scaling factors
        this.yspeed_mps = yspeed;
        this.turnspeed_radps = turnspeed;
    }

    private void AddDashboardEntrySwerveModule(SwerveModule module, String name) {
        // AddDashboardEntryWrite(name + " drive pos", 0.0, () -> {
        // return module.getDrivePosition();
        // });
        AddDashboardEntryWrite(name + " drive speed", 0.0, () -> {
            return module.getDriveVelocity();
        });
        // AddDashboardEntryWrite(name + " turn pos", 0.0, () -> {
        // return module.getTurningPosition() % (Math.PI * 2);
        // });
        AddDashboardEntryWrite(name + " abs enc", 0.0, () -> {
            return module.getAbsoluteEncoderRad();
        });
    }

    public void resetEncoders() {
        sm_fl.resetEncoders();
        sm_fr.resetEncoders();
        sm_bl.resetEncoders();
        sm_br.resetEncoders();
    }

    public void zeroEncoders() {
        sm_fl.zeroEncoders();
        sm_fr.zeroEncoders();
        sm_bl.zeroEncoders();
        sm_br.zeroEncoders();
    }

    public void formX() {
        sm_fl.setDesiredState(MathUtils.swerveModuleState(45, 0), false, true, true);
        sm_fr.setDesiredState(MathUtils.swerveModuleState(-45, 0), false, true, true);
        sm_bl.setDesiredState(MathUtils.swerveModuleState(-45, 0), false, true, true);
        sm_br.setDesiredState(MathUtils.swerveModuleState(45, 0), false, true, true);
    }

    public boolean isMoving(double thresh) {
        return Math.abs(sm_fl.getDriveVelocity()) > thresh
                || Math.abs(sm_bl.getDriveVelocity()) > thresh
                || Math.abs(sm_fr.getDriveVelocity()) > thresh
                || Math.abs(sm_br.getDriveVelocity()) > thresh;
    }

    private Swerve() {
        super("SWERVY BOI");

        globalState = GlobalState.getInstance();

        sm_br = cfg.swerve_br;
        sm_bl = cfg.swerve_bl;
        sm_fl = cfg.swerve_fl;
        sm_fr = cfg.swerve_fr;

        xlim = new SlewRateLimiter(8);
        ylim = new SlewRateLimiter(8);
        thetalim = new SlewRateLimiter(6 * Math.PI);

        kin = new SwerveDriveKinematics(
                new Translation2d(Constants.wheelBase_meters * 0.5, Constants.wheelBase_meters * 0.5), // fl
                new Translation2d(Constants.wheelBase_meters * 0.5, -Constants.wheelBase_meters * 0.5), // fr
                new Translation2d(-Constants.wheelBase_meters * 0.5, Constants.wheelBase_meters * 0.5), // bl
                new Translation2d(-Constants.wheelBase_meters * 0.5, -Constants.wheelBase_meters * 0.5) // br
        );

        kin2 = new DaveSwerveKin2(
                new Translation2d(Constants.wheelBase_meters * 0.5, Constants.wheelBase_meters * 0.5), // fl
                new Translation2d(Constants.wheelBase_meters * 0.5, -Constants.wheelBase_meters * 0.5), // fr
                new Translation2d(-Constants.wheelBase_meters * 0.5, Constants.wheelBase_meters * 0.5), // bl
                new Translation2d(-Constants.wheelBase_meters * 0.5, -Constants.wheelBase_meters * 0.5) // br
        );

        if (Preferences.getBoolean("comp", true)) {
            con = new DaveSwerveController(0.5, 0.0, 0.0,
                    0.13, 0.0, 0.0,
                    3.0, 0.0, 0.0,
                    0.1, 0.0, 0.00,
                    0.05, 0.0, 0.0, // LL
                    0.0, 0.0, // Antitilt
                    0, 0, 0 // balance
            );
        } else {
            con = new DaveSwerveController(0.5, 0.0, 0.0,
                    0.13, 0.0, 0.0,
                    AutoConstants.kPThetaController, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.05, 0.0, 0.0, // LL
                    0.2, 0.2, // Antitilt
                    0, 0, 0 // balance
            );
        }

        // con.setEnabled(false);

        pigeon = cfg.pigeon;
        odom = new SwerveDriveOdometry(kin2, Rotation2d.fromRadians(0),
                new SwerveModulePosition[] { sm_fl.odom(), sm_fr.odom(), sm_bl.odom(), sm_br.odom() });
        odomTrue = new SwerveDriveOdometry(kin2, Rotation2d.fromRadians(0),
                new SwerveModulePosition[] { sm_fl.odom(), sm_fr.odom(), sm_bl.odom(), sm_br.odom() });
        field = new Field2d();
        // fieldv = new Field2d();
        // pe = new SwerveDrivePoseEstimator(kin2, Rotation2d.fromRadians(0),
        // new SwerveModulePosition[] { sm_fl.odom(), sm_fr.odom(), sm_bl.odom(),
        // sm_br.odom() }, getPose(),
        // STATE_STDS, VISION_STDS);

        SmartDashboard.putData("Field", field);
        // SmartDashboard.putData("Field v", fieldv);

        holonomicTarget = new Rotation2d();
        targetPoint = new DavePoint(0, 0);
        targetConState = DaveSwerveController.defaultState;

        new Thread(() -> {
            try {
                Timer.delay(0.1);
                sm_fl.resetEncoders();
                Timer.delay(0.1);
                sm_fr.resetEncoders();
                Timer.delay(0.1);
                sm_bl.resetEncoders();
                Timer.delay(0.1);
                sm_br.resetEncoders();
                Timer.delay(0.1);
                zeroHeading();
                Timer.delay(0.1);

                zeroOdometry();
            } catch (Exception e) {
            }
        }).start();

        DISABLED = new State("DISABLED") {
            @Override
            public void init() {
                stopModules();

            }

            @Override
            public void periodic() {

            }
        };

        IDLE = new State("IDLE") {
            @Override
            public void init() {
            }

            @Override
            public void periodic() {
            }
        };

        ROBOT_RELATIVE = new State("ROBOT_RELATIVE") {
            @Override
            public void init() {
            }

            @Override
            public void periodic() {
                swerveDrive(xspeed_mps, yspeed_mps, turnspeed_radps, true);
            }
        };

        FIELD_RELATIVE_CLASSIC = new State("FIELD_RELATIVE_CLASSIC") {
            @Override
            public void init() {

            }

            @Override
            public void periodic() {
                // swerveDrive(xspeed_mps, yspeed_mps, turnspeed_radps);
                double xi = xlim.calculate(xspeed_mps);
                double yi = ylim.calculate(yspeed_mps);
                double ti = thetalim.calculate(turnspeed_radps);
                if (xspeed_mps == 0.0)
                    xi = 0.0;
                if (yspeed_mps == 0.0)
                    yi = 0.0;
                if (turnspeed_radps == 0.0)
                    ti = 0.0;
                // swerveDrive(xi, yi, ti);
                swerveDrive(xspeed_mps, yspeed_mps, turnspeed_radps);
            }
        };

        FIELD_RELATIVE_ABSOLUTE = new State("FIELD_RELATIVE_ABSOLUTE") {
            @Override
            public void init() {
            }

            @Override
            public void periodic() {
            }
        };

        SNAP_TOWARD = new State("SNAP_TOWARD") {
            @Override
            public void init() {

            }

            @Override
            public void periodic() {
                setModuleStatesFromChassisSpeeds(
                        con.calculate180(getPose(), new ChassisSpeeds(xspeed_mps, yspeed_mps, 0), Math.PI));
            }
        };
        SNAP_AWAY = new State("SNAP_AWAY") {
            @Override
            public void init() {

            }

            @Override
            public void periodic() {
                setModuleStatesFromChassisSpeeds(
                        con.calculate180(getPose(), new ChassisSpeeds(xspeed_mps, yspeed_mps, 0), 0));
            }
        };
        SNAP_HP = new State("SNAP HP") {
            @Override
            public void init() {

            }

            @Override
            public void periodic() {
                setModuleStatesFromChassisSpeeds(
                        con.calculate180(getPose(), new ChassisSpeeds(xspeed_mps, yspeed_mps, 0),
                                globalState.alliance() == Alliance.Red ? -Math.PI / 2 : Math.PI / 2));
            }
        };

        AUTO = new State("AUTO") {
            @Override
            public void init() {

            }

            @Override
            public void periodic() {
                setModuleStatesFromChassisSpeeds(con.calculate(getPose(), targetConState, getTargetHeading()));
            }
        };

        AUTO_TARGET_POINT = new State("AUTO_TARGET_POINT") {
            @Override
            public void init() {

            }

            @Override
            public void periodic() {
                // System.out.println(Math.toDegrees(targetPoint.radiansFrom(getPose().getX(),
                // getPose().getY())));
                setTargetHeading(targetPoint.radiansFrom(getPose().getX(), getPose().getY()));
                setModuleStatesFromChassisSpeeds(con.calculate(getPose(), targetConState, getTargetHeading()));
            }
        };

        AUTO_LL = new State("AUTO_LL") {
            boolean found_target = false;
            double targetY = 0;

            @Override
            public void init() {
                found_target = false;
                targetY = 0;
            }

            @Override
            public void periodic() {
                Pose2d cur = getPose();
                if (LL.TOP.validTarget() && Math.abs(LL.TOP.ty()) < 2.0) {
                    found_target = true;
                    targetY = cur.getY();
                }
                setModuleStatesFromChassisSpeeds(con.calculateLL(cur, targetConState, found_target, targetY));
            }
        };
        PREBALANCING_TOWARDS = new State("PREBALANCING_TOWARDS") {
            @Override
            public void init() {

            }

            @Override
            public void periodic() {
                double target_rad = MathUtils.closest90rad(Math.toRadians(pigeon.getHeadingDeg()));
                setModuleStatesFromChassisSpeeds(
                        ChassisSpeeds.fromFieldRelativeSpeeds(-2.5, 0.0,
                                con.getThetaCon().calculate(Math.toRadians(pigeon.getHeadingDeg()), target_rad),
                                pigeon.getRotation2d()));
            }
        };

        PREBALANCING_AWAY = new State("PREBALANCING_AWAY") {
            @Override
            public void init() {

            }

            @Override
            public void periodic() {
                double target_rad = MathUtils.closest90rad(Math.toRadians(pigeon.getHeadingDeg()));
                setModuleStatesFromChassisSpeeds(
                        ChassisSpeeds.fromFieldRelativeSpeeds(0.3, 0.0,
                                con.getThetaCon().calculate(pigeon.getHeadingDeg(), target_rad),
                                pigeon.getRotation2d()));
            }
        };

        BALANCING = new State("BALANCING") {
            double snapTo;

            @Override
            public void init() {
                if (Math.abs(pigeon.getHeadingDeg()) > 90) {
                    snapTo = 180;
                } else {
                    snapTo = 0;
                }
            }

            @Override
            public void periodic() {
                if (!con.isNaiveBalanaced()) {
                    setModuleStatesFromChassisSpeeds(
                            con.calculate180(getPose(), con.naiveBalance(), Math.toRadians(snapTo)));
                } else {
                    formX();
                }
            }
        };

        BASE_LOCK_PASSIVE = new State("BASE_LOCK_PASSIVE") {
            @Override
            public void init() {

            }

            @Override
            public void periodic() {
                formX();
            }
        };

        BASE_LOCK_ACTIVE = new State("BASE_LOCK_ACTIVE") {
            Pose2d pose;
            DaveSwerveController pid = new DaveSwerveController(
                    10, 0, 0, // x
                    10, 0, 0, // y
                    0.1, 0, 0, // theta
                    0.1, 0, 0, // head
                    0, 0, 0, // LL
                    0, 0, // antitilt
                    0, 0, 0 // balance
            );

            @Override
            public void init() {
                pose = getPose();
            }

            @Override
            public void periodic() {
                double dx = getPose().getX() - pose.getX();
                double dy = getPose().getY() - pose.getY();

                if (MathUtils.withinCircle(dx, dy, 0.03)) {
                    formX();
                } else {
                    swerveDrive(
                            pid.getXCon().calculate(dx),
                            pid.getYCon().calculate(dy),
                            0// pid.getXCon().calculate(measurement)
                    );
                }
            }
        };

        STTBOLLS = new State("STTBOLLS") {
            boolean close;

            @Override
            public void init() {
                close = true;
                con.resetThetaCon(getPose());
            }

            @Override
            public void periodic() {
                setModuleStatesFromChassisSpeeds(con.calculateLLClose(getPose()));
            }
        };

        STTBOLLS_AUTO = new State("STTBOLLS AUTO") {
            boolean close;

            @Override
            public void init() {
                close = true;
                con.resetThetaCon(getPose());
            }

            @Override
            public void periodic() {
                // if (con.atReference()) {
                // close = true;
                // }
                // if (close) {

                setModuleStatesFromChassisSpeeds(con.calculateLLClose(getPose()));
            }
        };

        STRAFING = new State("STRAFING") {
            @Override
            public void init() {
            }

            @Override
            public void periodic() {
                setModuleStatesFromChassisSpeeds(con.calculateLLFar(getPose()));
            }
        };

        SCORING = new State("SCORING") {
            @Override
            public void init() {
                con.resetThetaCon(getPose());
            }

            @Override
            public void periodic() {
                // TODO
            }
        };

        setCurrentState(DISABLED);

        AddDashboardEntryWrite("pigeon head", 0.0, pigeon::getHeadingDeg);
        AddDashboardEntryWrite("pigeon pitch", 0.0, pigeon::getPitch);
        AddDashboardEntryWrite("pigeon roll", 0.0, pigeon::getRoll);
        AddDashboardEntryWrite("distCS", 0.0, con::distanceCS);

        AddDashboardEntrySwerveModule(sm_fr, "FR");
        AddDashboardEntrySwerveModule(sm_fl, "FL");
        AddDashboardEntrySwerveModule(sm_bl, "BL");
        AddDashboardEntrySwerveModule(sm_br, "BR");

        // AddDashboardEntryWrite("vel", 0, () -> {
        // // return targetConState.vel_m_s;
        // return getMagnitude();
        // });

        // AddDashboardEntryWrite("pig vel", 0, pigeon::scuffedVelocityRadSec);

        AddDashboardEntryState(DISABLED);

    }

    public Pose2d getPose() {
        return odom.getPoseMeters();
    }

    public Pose2d getTruePose() {
        return odomTrue.getPoseMeters();
    }

    public void zeroHeading() {
        pigeon.resetHeading();
        lastHeading = pigeon.getHeadingDeg();
    }

    public void setHeading(double degrees) {
        pigeon.setYaw(degrees);
    }

    public void resetOdometry(Pose2d pose) {
        setHeading(pose.getRotation().getDegrees());
        odom.resetPosition(pose.getRotation(),
                new SwerveModulePosition[] { sm_fl.odom(), sm_fr.odom(), sm_bl.odom(), sm_br.odom() }, pose);
    }

    public void resetTrueOdometry(Pose2d pose) {
        // setHeading(pose.getRotation().getDegrees());
        odomTrue.resetPosition(pose.getRotation(),
                new SwerveModulePosition[] { sm_fl.odom(), sm_fr.odom(), sm_bl.odom(), sm_br.odom() }, pose);
    }

    public void zeroOdometry() {
        resetOdometry(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0.0)));
    }

    public void zeroTrueOdometry() {
        resetOdometry(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0.0)));
    }

    public Rotation2d getTargetHeading() {
        return holonomicTarget;
    }

    public void setTargetHeading(double degrees) {
        holonomicTarget = Rotation2d.fromDegrees(degrees);
    }

    public void setTargetHeadingRads(double radians) {
        holonomicTarget = new Rotation2d(radians);
    }

    public void setTargetPoint(DavePoint targetPoint) {
        this.targetPoint.x = targetPoint.x;
        this.targetPoint.y = targetPoint.y;
    }

    public void setTargetPoint(double x, double y) {
        targetPoint.x = x;
        targetPoint.y = y;
    }

    public void setTargetConState(DaveSwerveController.State desiredState) {
        targetConState = desiredState;
    }

    public void setTargetConState(Trajectory.State desiredState) {
        targetConState = new DaveSwerveController.State(desiredState.poseMeters, desiredState.velocityMetersPerSecond,
                desiredState.accelerationMetersPerSecondSq);
    }

    public void setTargetConState(Pose2d desiredPose, double vel_m_s, double acc_m_s2) {
        targetConState = new DaveSwerveController.State(desiredPose, vel_m_s, acc_m_s2);
    }

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    double lastHeading = 0;

    public void swerveDrive(double xSpeed, double ySpeed, double turnSpeed) {
        swerveDrive(xSpeed, ySpeed, turnSpeed, false);
    }

    public void swerveDrive(double xSpeed, double ySpeed, double turnSpeed, boolean robotRelative) {
        // ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed,
        // ySpeed, turnSpeed, new Rotation2d(0.0));
        double head = pigeon.getHeadingDeg();
        if (Math.abs(turnSpeed) < 0.01 && (xSpeed * xSpeed + ySpeed * ySpeed) > 0.001) {
            turnSpeed = con.headingCorrection(lastHeading, head);
        } else {
            lastHeading = head;
        }
        ChassisSpeeds s = con.calculate(getPose(), new ChassisSpeeds(xSpeed, ySpeed, turnSpeed), robotRelative);

        setModuleStatesFromChassisSpeeds(s);
    }

    public void setModuleStatesFromChassisSpeeds(ChassisSpeeds s) {
        // setModuleStates(kin.toSwerveModuleStates(s));

        // Cancels out skew from turning while strafing due to the swerve being a
        // time-varying system,
        // because a module can't instantly drive at a velocity the way we command it
        // to, the drivetrain
        // skews in the direction of turning while you strafe
        // Grand theft poofs code
        Pose2d robotPoseVel = new Pose2d(s.vxMetersPerSecond * Constants.controlDt_s,
                s.vyMetersPerSecond * Constants.controlDt_s,
                Rotation2d.fromRadians(s.omegaRadiansPerSecond * Constants.controlDt_s));
        Twist2d twist_vel = MathUtils.log(robotPoseVel);
        s = new ChassisSpeeds(twist_vel.dx / Constants.controlDt_s,
                twist_vel.dy / Constants.controlDt_s,
                twist_vel.dtheta / Constants.controlDt_s);

        // Anti tilt:
        // s = con.tiltCorrection(s, pigeon.getPitch(), pigeon.getRoll());

        chassisSpeeds = s;
        setModuleStates(kin2.toSwerveModuleStates(chassisSpeeds));
    }

    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }

    public double getMagnitude() {
        ChassisSpeeds s = kin2.toChassisSpeeds(sm_fl.getState(), sm_fr.getState(), sm_bl.getState(), sm_br.getState());
        return Math.sqrt(MathUtils.sq(s.vxMetersPerSecond) + MathUtils.sq(s.vyMetersPerSecond));
    }

    public TimestampedDoubleArray llPose() {
        if (LL.BOTTOM.validTarget()) {
            if (globalState.alliance() == Alliance.Red) {
                return LL.BOTTOM.botposeRed_timestamped();
            } else if (globalState.alliance() == Alliance.Blue) {
                return LL.BOTTOM.botposeBlu_timestamped();
            } else {
                return null;
            }
        } else {
            return null;
        }
    }

    @Override
    public void subsystemPeriodic() {
        Pose2d pose = odom.update(pigeon.getRotation2d(),
                new SwerveModulePosition[] { sm_fl.odom(), sm_fr.odom(), sm_bl.odom(), sm_br.odom() });
        odomTrue.update(pigeon.getRotation2d(),
                new SwerveModulePosition[] { sm_fl.odom(), sm_fr.odom(), sm_bl.odom(), sm_br.odom() });
        // pe.update(pigeon.getRotation2d(),
        // new SwerveModulePosition[] { sm_fl.odom(), sm_fr.odom(), sm_bl.odom(),
        // sm_br.odom() });
        // TimestampedDoubleArray llbpose = llPose();
        // if (llbpose != null) {
        // Pose2d visPose = LL.arrayToPose2d(llbpose.value);
        // Vector<N3> vissyStd = VISION_STDS;// VecBuilder.fill(0.5, 0.5,
        // Units.degreesToRadians(10));
        // pe.addVisionMeasurement(visPose, llbpose.serverTime, vissyStd);
        // }
        field.setRobotPose(pose);
        // fieldv.setRobotPose(pe.getEstimatedPosition());
        // SmartDashboard.putData("Field", field);
        // SmartDashboard.putData("Field v", fieldv);

        pigeon.periodic();
        // SmartDashboard.putNumber("pig vel", pigeon.scuffedVelocityRadSec());

        /*
         * if (recording) {
         * recordedPath.add(getPose());
         * }
         */
    }

    public void stopModules() {
        sm_fl.stop();
        sm_fr.stop();
        sm_bl.stop();
        sm_br.stop();
    }

    private void setModuleStates(DaveSwerveModuleState2[] desiredStates) {
        DaveSwerveKin2.desaturateWheelSpeeds(desiredStates, Constants.swerveTrueMaxSpeed_mps); // TODO
        sm_fl.setDesiredState(desiredStates[0]);
        sm_fr.setDesiredState(desiredStates[1]);
        sm_bl.setDesiredState(desiredStates[2]);
        sm_br.setDesiredState(desiredStates[3]);
        // SmartDashboard.putNumber("desired angle fl",
        // desiredStates[0].angle.getDegrees());
    }
}
