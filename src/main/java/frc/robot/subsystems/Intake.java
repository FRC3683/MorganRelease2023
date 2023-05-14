package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.config.Config;
import frc.robot.config.Constants;
import frc.robot.utils.GlobalState;
import frc.robot.utils.GlobalState.GamePiece;
import frc.robot.utils.EEMotion;

public class Intake extends DaveSubsystem {
    private static Intake instance;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    GlobalState globalState;
    boolean comp;

    private CANSparkMax mtr_roller;

    private Debouncer coneDebouncer;
    private Debouncer ballDebouncer;

    private final DigitalInput beam_cone;
    private final DigitalInput beam_cube;

    private double targetRollerSpeed;

    private EEMotion eemotion;

    public final State DISABLED, IDLE, INTAKECUBE, INTAKECONE, HOLD, RELEASECONE, RELEASECUBE, EEMOTIONAL, FIRE;

    public void setEEMotion(EEMotion input) {
        eemotion = input;
        setCurrentState(EEMOTIONAL);
    }

    public void stopMotors() {
        mtr_roller.stopMotor();
    }

    public boolean hasCone() {
        return !GlobalState.getInstance().disableConeBeam &&
                coneDebouncer.calculate(!beam_cone.get());
    }

    public boolean hasBall() {
        return !GlobalState.getInstance().disableBallBeam &&
                ballDebouncer.calculate(!beam_cube.get());
    }

    public void brakeMode() {
        mtr_roller.setIdleMode(IdleMode.kBrake);
    }

    public void coastMode() {
        mtr_roller.setIdleMode(IdleMode.kCoast);
    }

    public boolean hasAny() {
        return hasBall() || hasCone();
    }

    public boolean hasDesired() {
        if (globalState.gamepiece == GamePiece.CONE) {
            return hasCone();
        }
        return hasBall();
    }

    public double getTargetRollerSpeed() {
        return targetRollerSpeed;
    }

    private Intake() {
        super("INTAKE");

        globalState = GlobalState.getInstance();

        coneDebouncer = new Debouncer(Constants.conebeam_debounce);
        ballDebouncer = new Debouncer(Constants.ballbeam_debounce);

        targetRollerSpeed = 0.0;

        comp = Preferences.getBoolean("comp", true);

        var cfg = Config.getInstance();

        // eemotion = new EEMotion();

        mtr_roller = cfg.mtr_roller;

        beam_cone = cfg.beam_cone;
        beam_cube = cfg.beam_cube;

        // AddDashboardEntryWrite("current", false, () -> {
        // return mtr_roller.getOutputCurrent();
        // });

        AddDashboardEntryWrite("has cone", false, this::hasCone);
        AddDashboardEntryWrite("has ball", false, this::hasBall);
        // AddDashboardEntryWrite("target roller speed", 0.0,
        // this::getTargetRollerSpeed);

        DISABLED = new State("DISABLED") {
            @Override
            public void init() {
                stopMotors();
            }

            @Override
            public void periodic() {
            }
        };

        IDLE = new State("IDLE") {
            @Override
            public void init() {
                stopMotors();
                brakeMode();
                targetRollerSpeed = 0.0;
            }

            @Override
            public void periodic() {
            }
        };

        EEMOTIONAL = new State("EEMOTIONAL") {
            @Override
            public void init() {
                coastMode();
            }

            @Override
            public void periodic() {
                targetRollerSpeed = eemotion.getRollerSpeed();
                mtr_roller.set(targetRollerSpeed);

                if (eemotion.isFinished()) {
                    setCurrentState(IDLE);
                }
            }
        };

        INTAKECUBE = new State("INTAKECUBE") {
            @Override
            public void init() {
                // remember to throw this boolean in if preoaded with a cube
                mtr_roller.set(hasBall() ? Constants.ball_intake * 0.1 : Constants.ball_intake);
                brakeMode();
            }

            @Override
            public void periodic() {
            }
        };

        RELEASECUBE = new State("RELEASECUBE") {
            @Override
            public void init() {
                // remember to throw this boolean in if preloaded with a cub
                mtr_roller.set(Constants.ball_spit);
                brakeMode();
            }

            @Override
            public void periodic() {
                // sensor for later :)
            }
        };

        FIRE = new State("FIRE") {
            @Override
            public void init() {
                // remember to throw this boolean in if preloaded with a cub
                coastMode();
                mtr_roller.set(1.0);
            }

            @Override
            public void periodic() {
                // sensor for later :)
            }
        };

        RELEASECONE = new State("RELEASECONE") {
            @Override
            public void init() {
                mtr_roller.set(Constants.cone_spit);
                brakeMode();
            }

            @Override
            public void periodic() {
            }
        };

        INTAKECONE = new State("INTAKECONE") {
            @Override
            public void init() {
                mtr_roller.set(hasCone() ? Constants.cone_intake * 0.4 : Constants.cone_intake);
                brakeMode();
                mtr_roller.setSmartCurrentLimit(30);
            }

            @Override
            public void periodic() {
                // if(mtr_roller.getOutputCurrent() > 20) {

                // }
            }

            @Override
            public void onComplete() {
                mtr_roller.setSmartCurrentLimit(50);
            }
        };

        HOLD = new State("HOLD") {
            @Override
            public void init() {
                mtr_roller.setSmartCurrentLimit(2);
                brakeMode();
            }

            @Override
            public void periodic() {
                if (comp) {
                    if (hasCone() || (globalState.disableConeBeam && globalState.gamepiece == GamePiece.CONE)) {
                        mtr_roller.set(0.3);
                    } else if (hasBall() || (globalState.disableBallBeam && globalState.gamepiece == GamePiece.CUBE)) {
                        mtr_roller.set(-0.3);
                    } else {
                        mtr_roller.set(0);
                    }
                } else {
                    if (globalState.gamepiece == GamePiece.CUBE) {
                        mtr_roller.set(-0.3);
                    } else {
                        mtr_roller.set(0.3);
                    }
                }
            }

            @Override
            public void onComplete() {
                mtr_roller.setSmartCurrentLimit(50);
            }
        };

        setCurrentState(DISABLED);

        AddDashboardEntryState(DISABLED);
    }

    @Override
    public void subsystemPeriodic() {
    }
}