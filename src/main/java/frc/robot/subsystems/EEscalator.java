package frc.robot.subsystems;

import java.nio.file.DirectoryStream.Filter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.config.Config;
import frc.robot.config.Constants;
import frc.robot.utils.DaveLED;
import frc.robot.utils.EEMotion;
import frc.robot.utils.EEState;
import frc.robot.utils.EEscalator2d;
import frc.robot.utils.GlobalState;
import frc.robot.utils.MathUtils;
import frc.robot.utils.EEProfile.EEConstraints;
import frc.robot.utils.GlobalState.GamePiece;
import frc.robot.utils.GlobalState.ScoringLevel;

public class EEscalator extends DaveSubsystem {
    static EEscalator instance;

    public static EEscalator getInstance() {
        if (instance == null) {
            instance = new EEscalator();
        }
        return instance;
    }

    public final State DISABLED, ZEROING, CANCEL, EEMOTIONAL, INTAKING, STOWING, STOWED, HOLDING, OPENLOOP, MANUAL,
            TEST_EXTENDING,
            TEST_RETRACTING, IDLE,
            COAST;

    private double in_wristOpenLoop, in_linxOpenLoop, in_escOpenLoop;

    public final EEMotion ballLow, coneMid, ballMid, cone420, ball420, test, testDown, ballShoot, ballShootMid,
            ballShootHigh, coneShootMid, coneShootHigh, coneFree;
    public EEMotion eemotion;
    private EEState currentEEState = new EEState(0, 0, 0), eestateError = new EEState(0, 0, 0);
    public State postMotionState;

    public enum IntakePos {
        CONE,
        CUBE,
        NO_SPIN,
        HP,
        CUBE_AUTO
    }

    public IntakePos intakePos = IntakePos.CONE;

    private final CANSparkMax mtr_escalator;
    private final RelativeEncoder enc_escalator;
    private final SparkMaxPIDController pid_escalator;
    private final CANSparkMax mtr_linx;
    private final RelativeEncoder enc_linx;
    private final SparkMaxPIDController pid_linx;
    private final CANSparkMax mtr_wrist;
    private final RelativeEncoder enc_wrist;
    private final SparkMaxPIDController pid_wrist;
    // private final DaveLED led_left;
    // private final DaveLED led_right;

    EEscalator2d drawing;

    private final ElevatorFeedforward escFF;
    private final SimpleMotorFeedforward lnxFF;
    private final ArmFeedforward wrsFF;

    private final DigitalInput hall_sensorEsc;
    private final DigitalInput hall_sensorLinx;
    private final DutyCycleEncoder enc_absWrist;

    private final Intake intake;

    final int slot_holding = 0;
    final int slot_velocity = 1;
    final int slot_motion = 2;
    final int slot_holding_empty = 3;

    // the previous absolute encoder value, used to check whether the abs encoder is
    // disconnected or not
    private boolean startSet = false;
    private double previousAbs_deg = 0;
    private final double absRange = 5.0;

    public double wristManualOffset = 0.0;

    public boolean wristMoving;
    private boolean hasWristAbs = false;

    public boolean hasWristAbs() {
        return hasWristAbs;
    }

    private final GlobalState globalState;

    public void stopMotors() {
        wristMoving = false;
        mtr_linx.stopMotor();
        mtr_escalator.stopMotor();
        mtr_wrist.stopMotor();
    }

    public void openLoop(double in_wristOpenLoop, double in_linxOpenLoop, double in_escOpenLoop) {
        this.in_wristOpenLoop = in_wristOpenLoop;
        this.in_linxOpenLoop = in_linxOpenLoop;
        this.in_escOpenLoop = in_escOpenLoop;
    }

    private double manual_wst, manual_lnx, manual_esc;

    public void manual(double in_esc, double in_lnx, double in_wst) {
        manual_esc = in_esc;
        manual_lnx = in_lnx;
        manual_wst = in_wst;
    }

    void holdEsc(double esc) {
        pid_escalator.setReference(esc, ControlType.kPosition, slot_holding, escFF.calculate(0),
                ArbFFUnits.kVoltage);
    }

    void holdLnx(double lnx) {
        pid_linx.setReference(lnx, ControlType.kPosition, slot_holding, lnxFF.calculate(0));
    }

    void holdWst(double wst) {
        int slot = slot_holding_empty;
        if (!globalState.disableConeBeam && intake.hasCone()) {
            slot = slot_holding;
        }
        pid_wrist.setReference(wst, ControlType.kPosition, slot,
                wrsFF.calculate(Math.toRadians(wst), 0),
                ArbFFUnits.kVoltage);
        wristMoving = false;
    }

    public void hold(EEState eestate) {
        // eestateError = new EEState(currentEEState.escPos_m -
        // enc_escalator.getPosition(),
        // currentEEState.lnxPos_m - enc_linx.getPosition(),
        // currentEEState.wristPos_deg - enc_wrist.getPosition(),
        // currentEEState.escVel_mps - enc_escalator.getVelocity(),
        // currentEEState.lnxVel_mps - enc_linx.getVelocity(),
        // currentEEState.wristVel_degps - enc_wrist.getVelocity());
        currentEEState = eestate;
        holdEsc(eestate.escPos_m);
        holdLnx(eestate.lnxPos_m);
        holdWst(eestate.wristPos_deg);
    }

    public void followYourHeart(EEState es) {
        eestateError = new EEState(currentEEState.escPos_m - enc_escalator.getPosition(),
                currentEEState.lnxPos_m - enc_linx.getPosition(),
                currentEEState.wristPos_deg - enc_wrist.getPosition(),
                currentEEState.escVel_mps - enc_escalator.getVelocity(),
                currentEEState.lnxVel_mps - enc_linx.getVelocity(),
                currentEEState.wristVel_degps - enc_wrist.getVelocity());
        currentEEState = es;
        pid_escalator.setReference(es.escPos_m,
                ControlType.kPosition, slot_motion, escFF.calculate(es.escVel_mps, es.escAcc_mpsps),
                ArbFFUnits.kVoltage);
        pid_linx.setReference(es.lnxPos_m, ControlType.kPosition,
                slot_motion, lnxFF.calculate(es.lnxVel_mps, es.lnxAcc_mpsps),
                ArbFFUnits.kVoltage);
        double wpr = Math.toRadians(es.wristPos_deg);
        double wvr = Math.toRadians(es.wristVel_degps);
        double war = Math.toRadians(es.wristAcc_degpsps);
        pid_wrist.setReference(es.wristPos_deg, ControlType.kPosition,
                slot_motion, wrsFF.calculate(wpr, wvr, war), ArbFFUnits.kVoltage);
        wristMoving = true;
    }

    public void setEmotion(EEMotion eemotion, State postMotionState) {
        this.eemotion = eemotion;
        this.postMotionState = postMotionState;
        setCurrentState(EEMOTIONAL);
    }

    public boolean linxStowed() {
        return !hall_sensorLinx.get();
    }

    public boolean escStowed() {
        return !hall_sensorEsc.get();
    }

    public void zeroEscalator() {
        enc_escalator.setPosition(Constants.escalatorStowed_m);
    }

    public double wristAbs() {
        double value = (-enc_absWrist.get() * 360) % 360;
        if (value < -180) {
            value += 360;
        }
        if (value > 180) {
            value -= 360;
        }
        return value;
    }

    public void zeroWristAbs(double value) {
        if (!hasWristAbs) {
            return;
        }
        enc_wrist.setPosition(value);

    }

    public void zeroWristAbs() {
        zeroWristAbs(wristAbs());
    }

    public void zeroWristStowed() {
        enc_wrist.setPosition(Constants.wristStowed_deg);
    }

    public void zeroLinx() {
        enc_linx.setPosition(Constants.linxStowed_m);
    }

    private void setPidGains(SparkMaxPIDController pid, int slot, double kP, double kI, double kD, double iZone,
            double kFF, double min, double max) {
        pid.setP(kP, slot);
        pid.setI(kI, slot);
        pid.setD(kD, slot);
        pid.setIZone(iZone, slot);
        pid.setFF(kFF, slot);
        pid.setOutputRange(min, max, slot);
    }

    private void setSmartMotionConstants(SparkMaxPIDController pid, int slot, AccelStrategy strategy, double maxVel,
            double minVel,
            double maxAccel, double allowedErr) {
        pid.setSmartMotionMaxVelocity(maxVel, slot);
        pid.setSmartMotionMinOutputVelocity(minVel, slot);
        pid.setSmartMotionMaxAccel(maxAccel, slot);
        pid.setSmartMotionAllowedClosedLoopError(allowedErr, slot);
        pid.setSmartMotionAccelStrategy(strategy, slot);
    }

    public void setLEDs(int R, int G, int B) {
        // led_left.setColor(R, G, B);
        // led_right.setColour(R, G, B);
    }

    public EEMotion getMotionFrom(GamePiece gamepiece, ScoringLevel scoringLevel) {
        final EEMotion motions[][] = {
                { null, coneMid, cone420 },
                { ballLow, ballMid, ball420 }
        };
        return motions[gamepiece.index][scoringLevel.index];
    }

    private EEscalator() {
        super("EEscalator");

        intake = Intake.getInstance();

        globalState = GlobalState.getInstance();

        var cfg = Config.getInstance();

        wristMoving = false;

        hall_sensorEsc = cfg.hall_sensorEsc;
        hall_sensorLinx = cfg.hall_sensorLinx;
        enc_absWrist = cfg.enc_wristAbs;

        mtr_wrist = cfg.mtr_wrist;
        mtr_linx = cfg.mtr_linx;
        mtr_escalator = cfg.mtr_EEscalatorMaster;

        enc_wrist = mtr_wrist.getEncoder();
        enc_linx = mtr_linx.getEncoder();
        enc_escalator = mtr_escalator.getEncoder();

        enc_wrist.setVelocityConversionFactor(Constants.wristDegreesToRotations / 60);
        enc_wrist.setPositionConversionFactor(Constants.wristDegreesToRotations);

        enc_linx.setVelocityConversionFactor(Constants.linxMetersToRotations / 60);
        enc_linx.setPositionConversionFactor(Constants.linxMetersToRotations);

        enc_escalator.setVelocityConversionFactor(Constants.escalatorMetersToRotations / 60);
        enc_escalator.setPositionConversionFactor(Constants.escalatorMetersToRotations);

        pid_wrist = mtr_wrist.getPIDController();
        pid_linx = mtr_linx.getPIDController();
        pid_escalator = mtr_escalator.getPIDController();

        // led_left = new DaveLED(8, 21);
        // led_right = new DaveLED(9, 21);

        // if (Preferences.getBoolean("comp", true)) {
        setPidGains(pid_escalator, slot_holding, 3, 0.01, 0.0, 0.03, 0.0, -1, 1);
        setPidGains(pid_escalator, slot_velocity, 0.2, 0.0, 0.0, 0.0, 0.42, -1, 1);
        setSmartMotionConstants(pid_escalator, slot_velocity, AccelStrategy.kSCurve, 1.5, 0, 3, 0.03);
        // escFF = new ElevatorFeedforward(0, 0.34, 5.2, 0.35);
        escFF = new ElevatorFeedforward(0, Constants.esckG_V, Constants.esckV_Vspm, Constants.esckA_Vs2pm);
        setPidGains(pid_escalator, slot_motion, 5, 0.0, 0.0, 0.0, 0.0, -1, 1);

        setPidGains(pid_linx, slot_holding, 2.5, 0.05, 0, 0.02, 0.0, -1, 1);
        setPidGains(pid_linx, slot_velocity, 0.03, 0, 0, 0, 0.31, -1, 1);
        setSmartMotionConstants(pid_linx, slot_velocity, AccelStrategy.kSCurve, 0.2, 0, 0.5, 0.01);
        // lnxFF = new SimpleMotorFeedforward(0, 2.5, 0);
        lnxFF = new SimpleMotorFeedforward(0, Constants.lnxkV_Vspm * 0.5, Constants.lnxkA_Vs2pm);
        setPidGains(pid_linx, slot_motion, 4, 0.0, 0, 0.02, 0.0, -1, 1);

        setPidGains(pid_wrist, slot_holding, 0.013, 0.0, 0, 3, 0.0, -1, 1);
        setPidGains(pid_wrist, slot_holding_empty, 0.006, 0.0, 0.0, 3, 0.0, -1, 1);
        setPidGains(pid_wrist, slot_velocity, 0.0006, 0, 0, 0, 0.00125, -1, 1);
        setSmartMotionConstants(pid_wrist, slot_velocity, AccelStrategy.kSCurve, 90, 0, 500, 0.25);
        // wrsFF = new ArmFeedforward(0, 0.01 * Constants.nominalVoltage, 0, 0);
        wrsFF = new ArmFeedforward(0, 0.12, 0, Constants.wrskA_Vs2pdeg);
        setPidGains(pid_wrist, slot_motion, 0.023, 0.0, 0, 3, 0.0, -1, 1);

        // setPidGains(pid_linx, slot_holding, 23.0, 0.01, 0.0, 0.02, 0.0, -1, 1);
        // setPidGains(pid_linx, slot_velocity, 0.01, 0.0, 0.0, 0.0, 0.28, -1, 1);
        // setSmartMotionConstants(pid_linx, slot_velocity, AccelStrategy.kSCurve, 0.2,
        // 0, 0.5, 0.01);
        // lnxFF = new SimpleMotorFeedforward(0, 1.5, 0.1);
        // //lnxFF = new SimpleMotorFeedforward(0, 3.6, 0.55);
        // setPidGains(pid_linx, slot_motion, 11.0, 0.0, 0.0, 0.0, 0.0, -1, 1);

        // setPidGains(pid_wrist, slot_holding, 0.08, 0.0, 0.0, 0.0, 0.0, -1, 1);
        // setPidGains(pid_wrist, slot_velocity, 0.0003, 0.0, 0.0, 0.0, 0.0016, -1, 1);
        // setSmartMotionConstants(pid_wrist, slot_velocity, AccelStrategy.kSCurve, 45,
        // 0, 500, 0.25);
        // wrsFF = new ArmFeedforward(0, 0.14, 0.9, 0.12);
        // setPidGains(pid_wrist, slot_motion, 0.1, 0.0, 0.0, 0.0, 0.0, -1, 1);

        // } else { // practice bot

        // setPidGains(pid_escalator, slot_holding, 0.001, 0.0, 0, 0.02, 0.0, -1, 1);
        // setPidGains(pid_escalator, slot_velocity, 0.04, 0, 0, 0, 0.41, -1, 1);
        // setSmartMotionConstants(pid_escalator, slot_velocity, AccelStrategy.kSCurve,
        // 1.5, 0.0, 3, 0.03);
        // escFF = new ElevatorFeedforward(0, 0.023 * Constants.nominalVoltage, 0, 0);

        // setPidGains(pid_escalator, slot_motion, 6, 0, 0, 0, 0.0, -1, 1);

        // setPidGains(pid_linx, slot_holding, 2.5, 0.05, 0, 0.02, 0.0, -1, 1);
        // setPidGains(pid_linx, slot_velocity, 0.03, 0, 0, 0, 0.31, -1, 1);
        // setSmartMotionConstants(pid_linx, slot_velocity, AccelStrategy.kSCurve, 0.2,
        // 0, 0.5, 0.01);
        // lnxFF = new SimpleMotorFeedforward(0, 0, 0);
        // setPidGains(pid_linx, slot_motion, 3, 0.0, 0, 0.02, 0.0, -1, 1);

        // setPidGains(pid_wrist, slot_holding, 0.013, 0.0, 0, 3, 0.0, -1, 1);
        // setPidGains(pid_wrist, slot_velocity, 0.0006, 0, 0, 0, 0.00125, -1, 1);
        // setSmartMotionConstants(pid_wrist, slot_velocity, AccelStrategy.kSCurve, 45,
        // 0, 500, 0.25);
        // wrsFF = new ArmFeedforward(0, 0.01 * Constants.nominalVoltage, 0, 0);

        // setPidGains(pid_wrist, slot_motion, 0.023, 0.0, 0, 3, 0.0, -1, 1);
        // }

        EEState stowedEEState = new EEState(Constants.escalatorStowed_m, Constants.linxStowed_m,
                Constants.wristStowed_deg);

        EEConstraints testcon = new EEConstraints(2.1, 0.8, 300,
                8, 2.0, 1200,
                5000, 500, 50000);

        EEConstraints econMaxSmooth = new EEConstraints(2.1, 0.8, 300,
                Constants.escMaxAcc_mps2, Constants.lnxMaxAcc_mps2, Constants.wstMaxAcc_dps2,
                72, 72, 500000);

        EEConstraints econMaxSmoothShot = new EEConstraints(0.5, 0.8, 300,
                Constants.escMaxAcc_mps2, Constants.lnxMaxAcc_mps2, Constants.wstMaxAcc_dps2,
                72, 72, 500000);

        EEConstraints econMaxSmoothShotMid = new EEConstraints(1.0, 0.8, 300,
                Constants.escMaxAcc_mps2, Constants.lnxMaxAcc_mps2, Constants.wstMaxAcc_dps2,
                72, 72, 500000);

        EEConstraints econMaxSmoothShotHigh = new EEConstraints(2.1, 0.26, 300,
                Constants.escMaxAcc_mps2, Constants.lnxMaxAcc_mps2, Constants.wstMaxAcc_dps2,
                720, 72, 500000);

        EEConstraints econMaxJerky = new EEConstraints(2.1, 0.8, 300,
                8.0, Constants.lnxMaxAcc_mps2, Constants.wstMaxAcc_dps2,
                72, 72, 500000);

        // EEConstraints econIntake = new EEConstraints(2.1, 0.8, 600,
        // 8.0, Constants.lnxMaxAcc_mps2, 1000,
        // 72, 600, 500000);
        EEConstraints econIntake = new EEConstraints(2.1, 0.4, 600,
                8.0, 1.5, 900,
                72, 600, 500000);
        EEConstraints econStow = new EEConstraints(2.1, 0.8, 600,
                8.0, Constants.lnxMaxAcc_mps2, 1500,
                72, 300, 500000);

        EEConstraints econTest = new EEConstraints(1000, 1000, 10000, 1.5, 1.5, 300, 3, 3, 300);
        test = new EEMotion("TEST", econTest)
                .addEEState(stowedEEState)
                .addEEState(Constants.escalatorExtended_m, Constants.linxExtended_m, Constants.wristExtended_deg)
                .generate();
        testDown = new EEMotion("TEST DOWN", econTest)
                .addEEState(Constants.escalatorExtended_m, Constants.linxExtended_m, Constants.wristExtended_deg)
                .addEEState(stowedEEState)
                .generate();

        ballLow = new EEMotion("BALL LOW", econMaxJerky)
                .addEEState(Constants.ball_score_spit, stowedEEState)
                .addEEState(Constants.ball_score_spit, Constants.escalatorStowed_m, Constants.linxStowed_m, 55.0)
                .addEEState(stowedEEState)
                .generate();

        EEConstraints econConeMid = new EEConstraints(2.1, 0.8, 300,
                Constants.escMaxAcc_mps2, Constants.lnxMaxAcc_mps2, Constants.wstMaxAcc_dps2,
                72, 72, 500000);
        coneMid = new EEMotion("CONE MID", econConeMid)
                .addEEState(Constants.cone_hold, stowedEEState)
                .addEEState(Constants.cone_hold, 0.85, 0.49, 47)
                .addEEState(0, 0.5, 0.53, 20.5, -0.5, 0, 0)
                .addEEState(Constants.ball_score_spit, Constants.escalatorStowed_m, Constants.linxStowed_m,
                        Constants.wristStowed_deg, -0.1, 0, 0)
                .generate();
        EEConstraints econBallMid = new EEConstraints(2.1, 0.8, 300,
                Constants.escMaxAcc_mps2, 1, 300,
                72, 72, 500000);
        ballMid = new EEMotion("BALL MID", econBallMid)
                .addEEState(Constants.ball_hold, stowedEEState)
                .addEEState(Constants.ball_score_spit, 0.74, Constants.linxStowed_m, 61.0)
                // .addEEState(Constants.ball_score_spit, 0.74, Constants.linxStowed_m, 45.0)
                // .addEEState(stowedEEState)
                .addEEState(Constants.ball_score_spit, Constants.escalatorStowed_m, Constants.linxStowed_m,
                        Constants.wristStowed_deg, -0.1, 0, 0)
                .generate();

        EEConstraints econCone420 = new EEConstraints(2.1, 0.8, 300,
                Constants.escMaxAcc_mps2, Constants.lnxMaxAcc_mps2, 1000,
                140, 350, 50000);

        cone420 = new EEMotion("CONE 420", econCone420)
                .addEEState(Constants.cone_hold, stowedEEState)
                .addEEState(Constants.cone_hold, 0.83, Constants.linxStowed_m, Constants.wristStowed_deg, 1.8, 0, 0)
                .addEEState(0, 1.24, 0.595, 14, 0, 0, 0)
                .addEEState(-Constants.cone_hold, 1.16, 0.43, 14, -0.2, -0.5, 0)
                .addEEState(Constants.escalatorStowed_m, Constants.linxStowed_m, Constants.wristStowed_deg, -0.1, -0.5,
                        0)
                .generate();

        ballShoot = new EEMotion("BALL SHOOT", econMaxSmoothShot)
                .addEEState(Constants.ball_hold, stowedEEState)
                .addEEState(1, 0.42, Constants.linxStowed_m, Constants.wristStowed_deg, 0.5, 0,
                        0)
                .addEEState(1, 0.54, Constants.linxStowed_m, 65.0)
                .addEEState(stowedEEState)
                .generate();

        ballShootMid = new EEMotion("BALL SHOOT MID", econMaxSmoothShotHigh)
                .addEEState(Constants.ball_hold, stowedEEState)
                .addEEState(1, 0.60, Constants.linxStowed_m, Constants.wristStowed_deg, 2.1, 0,
                        0)
                .addEEState(1, 0.98, Constants.linxStowed_m, 65.0)
                .addEEState(Constants.escalatorStowed_m, Constants.linxStowed_m, Constants.wristStowed_deg, -0.1, -0.5,
                        0)
                .generate();

        ballShootHigh = new EEMotion("BALL SHOOT HIGH", econMaxSmoothShotHigh)
                .addEEState(Constants.ball_hold, stowedEEState)
                .addEEState(1, 0.85, Constants.linxStowed_m, Constants.wristStowed_deg, 2.1, 0,
                        0)
                .addEEState(1, 1.23, Constants.linxStowed_m, 65.0)
                .addEEState(Constants.escalatorStowed_m, Constants.linxStowed_m, Constants.wristStowed_deg, -0.1, -0.5,
                        0)
                .generate();

        coneShootMid = new EEMotion("CONE SHOOT MID", econMaxSmoothShotMid)
                .addEEState(Constants.cone_hold, stowedEEState)
                .addEEState(-1, 0.48, Constants.linxStowed_m, Constants.wristStowed_deg, 1.0, 0,
                        0)
                .addEEState(-1, 0.65, Constants.linxStowed_m, 65.0)
                .addEEState(Constants.escalatorStowed_m, Constants.linxStowed_m, Constants.wristStowed_deg, -0.1, -0.5,
                        0)
                .generate();

        coneShootHigh = new EEMotion("CONE SHOOT HIGH", econMaxSmoothShotHigh)
                .addEEState(Constants.cone_hold, stowedEEState)
                .addEEState(-1, 1.05, 0.492, Constants.wristStowed_deg, 2.1, 0.26,
                        0)
                .addEEState(-1, Constants.escalatorExtended_m, Constants.linxExtended_m, 65.0)
                .addEEState(Constants.escalatorStowed_m, Constants.linxStowed_m, Constants.wristStowed_deg, -0.1, -0.26,
                        0)
                .generate();

        coneFree = new EEMotion("CONE FREE", econMaxSmooth)
                .addEEState(Constants.cone_hold, stowedEEState)
                .addEEState(Constants.cone_hold, Constants.escalatorStowed_m, Constants.linxStowed_m + 0.02, 40)
                .addEEState(Constants.cone_hold, stowedEEState)
                .generate();

        EEConstraints econBall420 = new EEConstraints(2.1, 0.8, 300,
                Constants.escMaxAcc_mps2, 1, 300,
                72, 72, 500000);
        ball420 = new EEMotion("BALL 420", econBall420)
                .addEEState(Constants.ball_hold, stowedEEState)
                .addEEState(Constants.ball_score_spit, 1.05, 0.54, 61.0)
                // .addEEState(Constants.ball_score_spit, 1.0, 0.54, 60.95)
                .addEEState(Constants.escalatorStowed_m, Constants.linxStowed_m, Constants.wristStowed_deg, -0.1, -0.5,
                        0)
                .generate();

        AddDashboardEntryWrite("escalator pos", 0.0, enc_escalator::getPosition);
        // AddDashboardEntryWrite("escalator vel", 0.0, enc_escalator::getVelocity);
        AddDashboardEntryWrite("linx pos", 0.0, enc_linx::getPosition);
        // AddDashboardEntryWrite("linx vel", 0.0, enc_linx::getVelocity);
        AddDashboardEntryWrite("wrist pos", 0.0, enc_wrist::getPosition);
        // AddDashboardEntryWrite("wrist vel", 0.0, enc_wrist::getVelocity);
        AddDashboardEntryWrite("escalator hall", false, this::escStowed);
        AddDashboardEntryWrite("linx hall", false, this::linxStowed);
        AddDashboardEntryWrite("wrist abs", 0, this::wristAbs);
        AddDashboardEntryWrite("wrist abs raw", 0.0, enc_absWrist::get);

        AddDashboardEntryWrite("wrist flag", false, () -> {
            return hasWristAbs;
        });

        AddDashboardEntryWrite("state esc pos", 0, () -> {
            return currentEEState.escPos_m;
        });
        AddDashboardEntryWrite("state lnx pos", 0, () -> {
            return currentEEState.lnxPos_m;
        });
        AddDashboardEntryWrite("state wrist pos", 0, () -> {
            return currentEEState.wristPos_deg;
        });

        // AddDashboardEntryWrite("state esc vel", 0, () -> {
        // return currentEEState.escVel_mps;
        // });
        // AddDashboardEntryWrite("state lnx vel", 0, () -> {
        // return currentEEState.lnxVel_mps;
        // });
        // AddDashboardEntryWrite("state wrist vel", 0, () -> {
        // return currentEEState.wristVel_degps;
        // });

        // AddDashboardEntryWrite("state esc acc", 0, () -> {
        // return currentEEState.escAcc_mpsps;
        // });
        // AddDashboardEntryWrite("state lnx acc", 0, () -> {
        // return currentEEState.lnxAcc_mpsps;
        // });
        // AddDashboardEntryWrite("state wrist acc", 0, () -> {
        // return currentEEState.wristAcc_degpsps;
        // });

        AddDashboardEntryWrite("snapshot line", 0, () -> {
            return 300.0 * ((currentEEState.snapshot & 1) - 0.5);
        });

        AddDashboardEntryWrite("manual instake offset", 0, () -> {
            return wristManualOffset;
        });

        drawing = new EEscalator2d();

        DISABLED = new State("DISABLED") {
            @Override
            public void init() {
                wristMoving = false;
                stopMotors();
            }

            @Override
            public void periodic() {
                stopMotors();
            }
        };

        STOWED = new State("STOWED") {
            Timer timer = new Timer();
            MedianFilter filter = new MedianFilter((int) Math.ceil(2.5 / Constants.controlDt_s));
            double val = 0;
            boolean flag;
            private final EEState stowed = new EEState(Constants.escalatorStowed_m, Constants.linxStowed_m,
                    Constants.wristStowed_deg);

            @Override
            public void init() {
                wristMoving = false;
                currentEEState = stowedEEState;
                timer.reset();
                timer.start();
                filter.reset();
                flag = true;
            }

            @Override
            public void periodic() {
                val = filter.calculate(wristAbs());
                if (timer.get() > 2.5 && flag) { // BAD PWM ENCODER
                    zeroWristAbs(val);
                    flag = false;
                }
                hold(stowed);
            }
        };

        IDLE = new State("IDLE") {
            @Override
            public void init() {
                stopMotors();
            }

            @Override
            public void periodic() {
                stopMotors();
            }
        };

        ZEROING = new State("ZEROING") {

            @Override
            public void init() {
                wristMoving = true;
            }

            boolean wristStowed() {
                return MathUtils.closeEnough(enc_wrist.getPosition(), Constants.wristStowed_deg, 2.5);
            }

            @Override
            public void periodic() {
                zeroWristAbs();
                boolean escalatorZeroed = escStowed();
                if (escalatorZeroed) {
                    zeroEscalator();
                }
                boolean linxZeroed = linxStowed();
                if (linxZeroed) {
                    zeroLinx();
                }

                pid_escalator.setReference(-0.15, ControlType.kVelocity, slot_velocity,
                        escFF.calculate(-0.15),
                        ArbFFUnits.kVoltage);
                if (wristStowed()) {
                    holdWst(Constants.wristStowed_deg);
                } else {
                    pid_wrist.setReference(Constants.wristStowed_deg, ControlType.kSmartMotion,
                            slot_velocity, wrsFF.calculate(Math.toRadians(Constants.wristStowed_deg), 0),
                            ArbFFUnits.kVoltage);
                }
                double lnxSpeed = enc_wrist.getPosition() > 45 ? (linxZeroed ? -0.1 : -0.5) : 0;
                pid_linx.setReference(lnxSpeed, ControlType.kSmartVelocity, slot_velocity, lnxFF.calculate(lnxSpeed),
                        ArbFFUnits.kVoltage);

                if (RobotBase.isSimulation() || (wristStowed() && linxZeroed && escalatorZeroed)) {
                    zeroEscalator();
                    zeroLinx();
                    setCurrentState(STOWED);
                }
            }
        };

        CANCEL = new State("CANCEL") {
            private final EEConstraints econ = new EEConstraints(0, 0, 0, 0, 0, 0, 0, 0, 0);
            private EEMotion nineteen84;

            @Override
            public void init() {
                nineteen84 = new EEMotion("1984", econ)
                        .addEEState(currentEEState);
                // if (enc_linx.getPosition() > Constants.linxWristPoint) {
                // nineteen84.addEEState(Constants.escalatorStowed_m, Constants.linxWristPoint,
                // Constants.wristStowed_deg);
                // }
                nineteen84.addEEState(stowedEEState)
                        .generate();
                nineteen84.start();
            }

            @Override
            public void periodic() {
                followYourHeart(nineteen84.calculate());
                if (nineteen84.isFinished()) {
                    setCurrentState(ZEROING);
                }
            }
        };

        EEMOTIONAL = new State("EEMOTIONAL") {
            @Override
            public void init() {
                eemotion.reset();
                eemotion.start();
            }

            @Override
            public void periodic() {
                followYourHeart(eemotion.calculate());

                // SmartDashboard.putNumber("emotional damage",
                // eemotion.profiles.get(0).totalTime_s);

                if (eemotion.isFinished()) {
                    setCurrentState(postMotionState);
                }
            }
        };

        INTAKING = new State("INTAKING") {

            @Override
            public void init() {
                EEState intakeDownNoSpin = new EEState(Constants.escalatorStowed_m, 0.585, -7 + wristManualOffset);
                EEState intakeDownCone = new EEState(Constants.escalatorStowed_m, 0.585, -17 + wristManualOffset);
                // private final EEState intakeHPCone = new EEState(0.3356, 0.384, 61.6); // new
                // EEState(Constants.escalatorStowed_m + 0.04, Constants.linxStowed_m, 61.5);
                EEState intakeHPCone = new EEState(Constants.escalatorStowed_m + 0.045, Constants.linxStowed_m,
                        60 + wristManualOffset);
                EEState intakeDownBall = new EEState(Constants.escalatorStowed_m, 0.585, -5.5
                        + wristManualOffset);
                // EEState intakeDownBall = new EEState(Constants.escalatorStowed_m + 0.05,
                // 0.585,
                // -7.5 + wristManualOffset);
                EEState intakeDownBallAuto = new EEState(Constants.escalatorStowed_m, Constants.linxExtended_m,
                        -6.67 + wristManualOffset);
                eemotion = new EEMotion("INTAKE", intakePos == IntakePos.HP ? econMaxSmooth : econIntake);
                eemotion.addEEState(currentEEState.escPos_m, currentEEState.lnxPos_m, currentEEState.wristPos_deg);
                switch (intakePos) {
                    case CONE:
                        eemotion.addEEState(intakeDownCone);
                        break;
                    case CUBE:
                        eemotion.addEEState(intakeDownBall);
                        break;
                    case NO_SPIN:
                        eemotion.addEEState(intakeDownNoSpin);
                        break;
                    case HP:
                        globalState.gamepiece = GamePiece.CONE;
                        eemotion.addEEState(intakeHPCone);
                        break;
                    case CUBE_AUTO:
                        eemotion.addEEState(intakeDownBallAuto);
                        break;
                }
                eemotion.generate();
                eemotion.start();
            }

            @Override
            public void periodic() {
                if (eemotion.isFinished()) {
                    // hold(eemotion.finalState());
                } else {
                    followYourHeart(eemotion.calculate());
                }
            }
        };

        STOWING = new State("STOWING") {
            @Override
            public void init() {
                if (intakePos == IntakePos.HP) {
                    setCurrentState(ZEROING);
                    return;
                }
                eemotion = new EEMotion("STOWING", econStow);
                eemotion.addEEState(currentEEState.escPos_m, currentEEState.lnxPos_m, currentEEState.wristPos_deg);
                if (intakePos != IntakePos.HP && currentEEState.wristPos_deg < Constants.wristLimit
                        && currentEEState.lnxPos_m > Constants.linxLimit) {
                    eemotion.addEEState(Constants.escalatorStowed_m, currentEEState.lnxPos_m,
                            Constants.wristLimit, 0, 0,
                            MathUtils.remap(
                                    Constants.wristExtended_deg,
                                    Constants.wristLimit,
                                    currentEEState.wristPos_deg,
                                    280, 0));
                }
                eemotion.addEEState(Constants.escalatorStowed_m, Constants.linxStowed_m, Constants.wristStowed_deg, 0,
                        -0.5, 0);
                eemotion.generate();
                eemotion.reset();
                eemotion.start();
            }

            @Override
            public void periodic() {
                followYourHeart(eemotion.calculate());

                if (eemotion.isFinished()) {
                    setCurrentState(ZEROING);
                }
            }
        };

        MANUAL = new State("MANUAL") {
            double esc, wst, lnx;
            boolean escMoving, lnxMoving, wstMoving;

            @Override
            public void init() {
                esc = enc_escalator.getPosition();
                lnx = enc_linx.getPosition();
                wst = enc_wrist.getPosition();
            }

            @Override
            public void periodic() {
                escMoving = escMoving || (manual_esc != 0);
                lnxMoving = lnxMoving || (manual_lnx != 0);
                wstMoving = wstMoving || (manual_wst != 0);
                if (escMoving) {
                    if (manual_esc == 0) {
                        escMoving = false;
                        esc = enc_escalator.getPosition();
                    } else {
                        pid_escalator.setReference(manual_esc, ControlType.kVelocity, slot_velocity,
                                escFF.calculate(manual_esc),
                                ArbFFUnits.kVoltage);
                    }
                } else {
                    holdEsc(esc);
                }
                if (lnxMoving) {
                    if (manual_lnx == 0) {
                        lnxMoving = false;
                        lnx = enc_linx.getPosition();
                    } else {
                        pid_linx.setReference(manual_lnx, ControlType.kSmartVelocity, slot_velocity,
                                lnxFF.calculate(manual_lnx),
                                ArbFFUnits.kVoltage);
                    }
                } else {
                    holdLnx(lnx);
                }
                if (wstMoving) {
                    if (manual_wst == 0) {
                        wstMoving = false;
                        wst = enc_wrist.getPosition();
                    } else {
                        pid_wrist.setReference(manual_wst, ControlType.kSmartVelocity,
                                slot_velocity, wrsFF.calculate(Math.toRadians(enc_wrist.getPosition()), 0),
                                ArbFFUnits.kVoltage);
                    }
                } else {
                    holdWst(wst);
                }
            }
        };

        HOLDING = new State("HOLDING") {
            private EEState start;

            @Override
            public void init() {
                start = new EEState(enc_escalator.getPosition(), enc_linx.getPosition(), enc_wrist.getPosition());
            }

            @Override
            public void periodic() {
                hold(start);
            }
        };

        OPENLOOP = new State("OPEN LOOP") {
            @Override
            public void init() {
            }

            @Override
            public void periodic() {
                mtr_wrist.set(in_wristOpenLoop);
                mtr_linx.set(in_linxOpenLoop);
                mtr_escalator.set(in_escOpenLoop);
            }
        };

        TEST_EXTENDING = new State("TEST EXTENDING") {
            EEMotion test;

            @Override
            public void init() {
                EEConstraints testcon = new EEConstraints(0.5, 0.8, 300,
                        1, 1.0, 1200,
                        50, 50, 500);
                // EEConstraints testcon = new EEConstraints(2.0, 0.8, 1200,
                // 6.0, 2.0, 2400,
                // 50, 50, 500);
                test = new EEMotion("test", testcon);
                test.addEEState(stowedEEState)
                        .addEEState(Constants.escalatorStowed_m, Constants.linxStowed_m, Constants.wristStowed_deg)
                        .addEEState(1.22, Constants.linxStowed_m, Constants.wristStowed_deg)
                        .addEEState(1.22, 0.595, Constants.wristStowed_deg)
                        .addEEState(1.22, 0.595, 14)
                        .generate();
                test.start();
            }

            @Override
            public void periodic() {
                // pid_escalator.setReference(1, ControlType.kVelocity, slot_velocity);
                followYourHeart(test.calculate());

                if (test.isFinished()) {
                    setCurrentState(HOLDING);
                }
            }
        };

        TEST_RETRACTING = new State("TEST RETRACTING") {
            @Override
            public void init() {
            }

            @Override
            public void periodic() {
            }
        };

        COAST = new State("COAST") {
            CANSparkMax slave;

            @Override
            public void init() {
                slave = Config.getInstance().mtr_EEscalatorSlave;
                mtr_escalator.setIdleMode(IdleMode.kCoast);
                slave.setIdleMode(IdleMode.kCoast);
                mtr_linx.setIdleMode(IdleMode.kCoast);
                mtr_wrist.setIdleMode(IdleMode.kCoast);
            }

            @Override
            public void onComplete() {
                mtr_escalator.setIdleMode(IdleMode.kBrake);
                slave.setIdleMode(IdleMode.kBrake);
                mtr_linx.setIdleMode(IdleMode.kBrake);
                mtr_wrist.setIdleMode(IdleMode.kBrake);
            }

            @Override
            public void periodic() {
            }
        };

        setCurrentState(DISABLED);

        AddDashboardEntryState(DISABLED);
    }

    public EEState getEEState() {
        return new EEState(enc_escalator.getPosition(),
                enc_linx.getPosition(),
                enc_wrist.getPosition(),
                enc_escalator.getVelocity(),
                enc_linx.getVelocity(),
                enc_wrist.getVelocity());
    }

    private double prev = 0;
    private Debouncer hasWristDeounce = new Debouncer(Constants.controlDt_s * 5);

    @Override
    public void subsystemPeriodic() {
        drawing.drawMeasured(enc_escalator.getPosition(), enc_linx.getPosition(),
                enc_wrist.getPosition());
        drawing.drawTarget(currentEEState.escPos_m, currentEEState.lnxPos_m,
                currentEEState.wristPos_deg);

        double curr = enc_absWrist.get();
        hasWristAbs = !hasWristDeounce.calculate(curr == prev);
        prev = curr;
    }

}
