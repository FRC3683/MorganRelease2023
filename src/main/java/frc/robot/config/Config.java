package frc.robot.config;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.DavePigeon;
import frc.robot.utils.Primes;

public class Config {

    public static boolean performanceMode = false;
    public static boolean burnFlash = false;
    public static boolean flashHasBeenBurned = true;
    public static boolean setPrefsComp = false;
    public static boolean setPrefsPractice = false;

    // CAN
    public static final int id_rio = 0;
    public static final int id_pdh = 1;
    public static final int id_swerve_br_turn = 2;
    public static final int id_swerve_bl_turn = 3;
    public static final int id_swerve_fl_turn = 4;
    public static final int id_swerve_fr_turn = 5;
    public static final int id_swerve_br_drive = 6;
    public static final int id_swerve_bl_drive = 7;
    public static final int id_swerve_fl_drive = 8;
    public static final int id_swerve_fr_drive = 9;
    public static final int id_pigeon = 10;
    public static final int id_elevator_master = 11;
    public static final int id_elevator_slave = 12;
    public static final int id_linx = 13;
    public static final int id_wrist = 14;
    public static final int id_intake = 15;
    public final CANSparkMax mtr_EEscalatorSlave;
    public final CANSparkMax mtr_EEscalatorMaster;
    public final CANSparkMax mtr_linx;
    public final CANSparkMax mtr_wrist;
    public final CANSparkMax mtr_roller;

    // DIO
    public static final int hallEsc_id = 0;
    public static final int hallLinx_id = 1;
    public static final int id_wristAbs = 7;
    public static final int id_beam_cone = 2;
    public static final int id_beam_cube = 3;
    public final DigitalInput hall_sensorEsc;
    public final DigitalInput hall_sensorLinx;
    public final DigitalInput beam_cone;
    public final DigitalInput beam_cube;
    public final DutyCycleEncoder enc_wristAbs;

    private static final int id_abs_br = 9;
    private static final int id_abs_bl = 5;
    private static final int id_abs_fl = 8;
    private static final int id_abs_fr = 4;

    private static final int driveCurrentLimitA = 30, turnCurrentLimitA = 40;

    CANSparkMax configSpark(int id, int smartCurrentLimit, IdleMode idleMode, boolean reversed, int frame0Period_ms,
            int frame1Period_ms, int frame2Period_ms) {
        CANSparkMax spark = new CANSparkMax(id, MotorType.kBrushless);
        if (!flashHasBeenBurned) {
            spark.clearFaults();
            spark.restoreFactoryDefaults(false);
            spark.enableVoltageCompensation(Constants.nominalVoltage);
            spark.setSmartCurrentLimit(smartCurrentLimit); // TODO: better current limiting
            spark.setIdleMode(idleMode);
            spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, frame0Period_ms);
            spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, frame1Period_ms);
            spark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, frame2Period_ms);
            Timer.delay(0.2);
            spark.setInverted(reversed);
        }
        return spark;
    }

    private Config() {

        if (setPrefsComp) {
            Preferences.initBoolean("comp", true);
        } else if (setPrefsPractice) {
            Preferences.initBoolean("comp", false);
        }

        boolean comp = Preferences.getBoolean("comp", true);

        if (comp) {

            swerve_br = new SwerveModule(
                    id_swerve_br_drive,
                    id_swerve_br_turn,
                    false,
                    true,
                    id_abs_br,
                    -0.66, // abs enc offset
                    false,
                    driveCurrentLimitA, turnCurrentLimitA);

            swerve_bl = new SwerveModule(
                    id_swerve_bl_drive,
                    id_swerve_bl_turn,
                    false,
                    true,
                    id_abs_bl,
                    -3.615,//-1.57, // abs enc offset
                    false,
                    driveCurrentLimitA, turnCurrentLimitA);

            swerve_fl = new SwerveModule(
                    id_swerve_fl_drive,
                    id_swerve_fl_turn,
                    false,
                    true,
                    id_abs_fl,
                    -0.93, // abs enc offset
                    false,
                    driveCurrentLimitA, turnCurrentLimitA);

            swerve_fr = new SwerveModule(
                    id_swerve_fr_drive,
                    id_swerve_fr_turn,
                    true,
                    true,
                    id_abs_fr, // abs enc id
                    -2.13,
                    false,
                    driveCurrentLimitA, turnCurrentLimitA);

        } else {

            swerve_br = new SwerveModule(
                    id_swerve_br_drive,
                    id_swerve_br_turn,
                    false,
                    true,
                    id_abs_br,
                    -0.189, // abs enc offset
                    false,
                    driveCurrentLimitA, turnCurrentLimitA);

            swerve_bl = new SwerveModule(
                    id_swerve_bl_drive,
                    id_swerve_bl_turn,
                    true,
                    true,
                    id_abs_bl,
                    -0.48, // abs enc offset
                    false,
                    driveCurrentLimitA, turnCurrentLimitA);

            swerve_fl = new SwerveModule(
                    id_swerve_fl_drive,
                    id_swerve_fl_turn,
                    true,
                    true,
                    id_abs_fl,
                    -1.690, // abs enc offset
                    false,
                    driveCurrentLimitA, turnCurrentLimitA);

            swerve_fr = new SwerveModule(
                    id_swerve_fr_drive,
                    id_swerve_fr_turn,
                    true,
                    true,
                    id_abs_fr, // abs enc id
                    -2.920,
                    false,
                    driveCurrentLimitA, turnCurrentLimitA);
        }

        pigeon = new DavePigeon(id_pigeon, false, -90);

        mtr_EEscalatorSlave = configSpark(id_elevator_master, 60, IdleMode.kBrake, true, 10, 10, 10);
        mtr_EEscalatorMaster = configSpark(id_elevator_slave, 60, IdleMode.kBrake, true, 10, 10, 10);
        mtr_EEscalatorSlave.follow(mtr_EEscalatorMaster, true);
        mtr_linx = configSpark(id_linx, 55, IdleMode.kBrake, false, 10, 10, 10);
        mtr_wrist = configSpark(id_wrist, 30, IdleMode.kBrake, false, 10, 10, 10);
        mtr_roller = configSpark(id_intake, 30, IdleMode.kCoast, false, 10, 10, Primes.next());
        hall_sensorEsc = new DigitalInput(hallEsc_id);
        hall_sensorLinx = new DigitalInput(hallLinx_id);
        enc_wristAbs = new DutyCycleEncoder(id_wristAbs);
        enc_wristAbs.setDutyCycleRange(1.0 / 4096, 4095.0 / 4096);
        enc_wristAbs.setPositionOffset(comp ? 0.598 : 0.362);
        beam_cone = new DigitalInput(id_beam_cone);
        beam_cube = new DigitalInput(id_beam_cube);

        // Callibration delay
        if (burnFlash) {
            swerve_bl.burnFlash();
            swerve_br.burnFlash();
            swerve_fl.burnFlash();
            swerve_fr.burnFlash();
            mtr_EEscalatorMaster.burnFlash();
            mtr_EEscalatorSlave.burnFlash();
            mtr_linx.burnFlash();
            mtr_wrist.burnFlash();
            mtr_roller.burnFlash();
            try {
                Thread.sleep(1000, 0);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    // HW
    public final SwerveModule swerve_fl, swerve_fr, swerve_br, swerve_bl;
    public final DavePigeon pigeon;

    private static Config instance;

    public static Config getInstance() {
        if (instance == null) {
            instance = new Config();
        }
        return instance;
    }
}
