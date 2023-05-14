package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.AutoSelector;
import frc.robot.utils.DaveLED;
import frc.robot.utils.GlobalState;
import frc.robot.utils.LL;
import frc.robot.utils.MathUtils;
import frc.robot.utils.OI;
import frc.robot.utils.Xkey60;
import frc.robot.utils.GlobalState.GamePiece;

public class LEDs extends SubsystemBase {
    private static LEDs instance;

    public static LEDs getInstance(RobotBase robot) {
        if (instance == null) {
            instance = new LEDs(robot);
        }
        return instance;
    }

    private final DaveLED strip;
    private final Swerve swerve;
    private final Timer timer;
    private final GlobalState globalState;
    private final Xkey60 xkeys;
    private final Intake intake;
    private final EEscalator escalator;
    private final AutoSelector selector;
    private final OI oi;

    private final Color8Bit clr_alliance_non = new Color8Bit(200, 200, 200);
    private final Color8Bit clr_alliance_red = new Color8Bit(200, 0, 0);
    private final Color8Bit clr_alliance_blu = new Color8Bit(0, 0, 200);
    private final Color8Bit clr_cone = new Color8Bit(255, 255, 0);
    private final Color8Bit clr_cube = new Color8Bit(106, 13, 173);
    private final Color8Bit clr_intake_has = new Color8Bit(255, 0, 0);
    private final Color8Bit clr_sttbolls = new Color8Bit(200, 85, 0);
    private final Color8Bit clr_sttbolls_ready = new Color8Bit(0, 255, 0);

    private final Color8Bit clr_bopit_red = new Color8Bit(255, 0, 0);
    private final Color8Bit clr_bopit_blue = new Color8Bit(0, 0, 255);
    private final Color8Bit clr_bopit_yellow = new Color8Bit(255, 255, 0);
    private final Color8Bit clr_bopit_green = new Color8Bit(0, 255, 0);
    private final Color8Bit clr_bopit_white = new Color8Bit(255, 255, 255);

    private boolean bopping_it;
    private final Timer bopping_timer;
    private Color8Bit bopit_color;
    private int score;

    private LEDs(RobotBase robot) {
        strip = new DaveLED(1, Preferences.getBoolean("comp", true) ? 18 : 36);
        swerve = Swerve.getInstance();
        timer = new Timer();
        timer.start();
        globalState = GlobalState.getInstance();
        xkeys = Xkey60.getInstance();
        intake = Intake.getInstance();
        escalator = EEscalator.getInstance();
        selector = AutoSelector.getInstance();
        oi = OI.getInstance();

        bopping_it = false;
        bopping_timer = new Timer();
    }

    private void bopit() {
        bopping_it = true;
        double num = Math.random();
        if(num < 0.25) {
            bopit_color = clr_bopit_red;
        } else if(num < 0.5) {
            bopit_color = clr_bopit_green;
        } else if(num < 0.75) {
            bopit_color = clr_bopit_blue;
        } else {
            bopit_color = clr_bopit_yellow;
        }
        bopping_timer.restart();
        SmartDashboard.putNumber("bopit", score);
    }

    private void lose() {
        bopping_it = false;
        SmartDashboard.putNumber("bopit", score);
    }

    @Override
    public void periodic() {
        strip.wipe();

        int pos0 = (int) Math.round((Math.sin(timer.get() * 8) * 0.5 + 0.5) * (strip.length - 1));
        int pos1 = (int) Math.round((Math.cos(timer.get() * 6) * 0.5 + 0.5) * (strip.length - 1));
        if (DriverStation.isDisabled()) {
            if (!bopping_it && oi.getAButtonPressedDriver() && MathUtils.closeEnough(pos1, pos0, 1)) {
                bopit();
                score = 0;
            }
            if(bopping_it) {
                double time = bopping_timer.get();
                if(time < 0.1) {
                    strip.setColor(clr_bopit_white);
                } else if(time > 1 && time < 1.1) {
                    strip.setColor(bopit_color);
                } else if(time > 1 && time < 2) {
                    if(oi.getAButtonDriver()) {
                        if(bopit_color == clr_bopit_green) {
                            ++score;
                            bopit();
                        } else {
                            lose();
                        }
                    } else if(oi.getBButtonDriver()) {
                        if(bopit_color == clr_bopit_red) {
                            ++score;
                            bopit();
                        } else {
                            lose();
                        }
                    } else if(oi.getXButtonDriver()) {
                        if(bopit_color == clr_bopit_blue) {
                            ++score;
                            bopit();
                        } else {
                            lose();
                        }
                    } else if(oi.getYButtonDriver()) {
                        if(bopit_color == clr_bopit_yellow) {
                            ++score;
                            bopit();
                        } else {
                            lose();
                        }
                    }
                } else if(time > 2) {
                    lose();
                }
            }
        }

        boolean topComms = true;//LL.TOP.comms();
        boolean bottomComms = true;//LL.BOTTOM.comms();
        boolean hasWrist = true;//escalator.hasWristAbs();

        if (xkeys.flashCone() || xkeys.flashCube()) {
            if (timer.get() % 0.2 < 0.1) {
            } else if (xkeys.flashCone()) {
                strip.setColor(clr_cone);
            } else {
                strip.setColor(clr_cube);
            }
        } else if (swerve.STTBOLLS_AUTO.isActive() && swerve.con.readyToScore(swerve.getPose())) {
            if (timer.get() % 0.2 < 0.1) {
                strip.setColor(clr_sttbolls_ready);
            }
        } else if ((intake.hasCone() || intake.hasBall())
                && escalator.INTAKING.isActive()) {
            if (timer.get() % 0.2 < 0.1) {
                strip.setColor(strip.clr_off);
            } else {
                strip.setColor(clr_intake_has);
            }
        } else if (!topComms || !bottomComms || !hasWrist) {
            boolean alternate = timer.get() * 2 % 1 > 0.5;
            if (!hasWrist) {
                strip.setCheckerErr(0, 7, alternate);
            } else {
                strip.setColor(0, 7, strip.clr_off);
            }
            if (!bottomComms) {
                strip.setCheckerErr(7, 13, alternate);
            } else {
                strip.setColor(7, 13, strip.clr_off);
            }
            if (!topComms) {
                strip.setCheckerErr(13, 20, alternate);
            } else {
                strip.setColor(13, 20, strip.clr_off);
            }
        } else if (swerve.STTBOLLS.isActive()) {
            strip.setColor(clr_sttbolls);
        } else if (DriverStation.isDisabled()) {
        } else if (globalState.gamepiece == GamePiece.CUBE) {
            strip.setColorScoringLevel(globalState.scoringLevel, clr_cube);
        } else {
            strip.setColorScoringLevel(globalState.scoringLevel, clr_cone);
        }

        if (DriverStation.isDisabled()) {
            Color8Bit alliance = clr_alliance_non;
            if (DriverStation.getAlliance() == Alliance.Blue) {
                alliance = clr_alliance_blu;
            } else if (DriverStation.getAlliance() == Alliance.Red) {
                alliance = clr_alliance_red;
            }
            // strip.setColor(7, 11, selector.R(), selector.G(), selector.B());
            strip.setTrail(pos0, alliance);
            strip.setTrail(pos1, selector.color());

            if (selector.generated()) {
                strip.setSingle(0, selector.generatedColor());
                strip.setSingle(strip.length - 1, selector.generatedColor());
            }
        }

        strip.drawLEDs();
    }
}
