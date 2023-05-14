package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Controls;
import frc.robot.subsystems.EEscalator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.EEscalator.IntakePos;
import frc.robot.utils.EEMotion;
import frc.robot.utils.GlobalState;
import frc.robot.utils.LL;
import frc.robot.utils.OI;
import frc.robot.utils.Xkey60;
import frc.robot.utils.GlobalState.GamePiece;
import frc.robot.utils.GlobalState.ScoringLevel;

public class RobotTeleop extends CommandBase {

    private final OI oi;
    private static Xkey60 xkeys;
    private final GlobalState globalState;
    private final Swerve swerve;
    private final EEscalator escalator;
    private final Intake intake;

    boolean manualSuperstructure = false;

    public RobotTeleop() {
        oi = OI.getInstance();
        xkeys = Xkey60.getInstance();
        globalState = GlobalState.getInstance();
        swerve = Swerve.getInstance();
        escalator = EEscalator.getInstance();
        intake = Intake.getInstance();
        addRequirements(swerve, escalator, intake);
    }

    @Override
    public void initialize() {
        escalator.setCurrentState(escalator.ZEROING);
        intake.setCurrentState(intake.IDLE);
        swerve.setCurrentState(swerve.FIELD_RELATIVE_CLASSIC);
    }

    private void emotion(EEMotion motion, EEscalator.State postMotionState) {
        escalator.setEmotion(motion, postMotionState);
        intake.setEEMotion(motion);
    }

    void score() {
        if (!escalator.STOWED.isActive()) {
            return;
        }
        EEMotion motion = escalator.getMotionFrom(globalState.gamepiece, globalState.scoringLevel);
        if (motion != null) {
            emotion(motion, escalator.ZEROING);
        }
    }

    void shoot() {
        if (!escalator.STOWED.isActive()) {
            return;
        }
        EEMotion motion = globalState.gamepiece == GamePiece.CONE ? escalator.coneShootHigh
                : escalator.ballShootHigh;
        if (motion != null) {
            emotion(motion, escalator.ZEROING);
        }
    }

    @Override
    public void execute() {

        globalState.scoringLevel = xkeys.scoringLevel();
        globalState.scoringSlot = xkeys.scoringSlot();
        globalState.scoringOffset = xkeys.scoringOffset();
        // SmartDashboard.putString("scoring slot", globalState.scoringSlot.toString());
        // SmartDashboard.putString("scoring level",
        // globalState.scoringLevel.toString());
        // SmartDashboard.putString("gamepiece", globalState.gamepiece.toString());

        if (xkeys.wristOffsetDown()) {
            escalator.wristManualOffset -= 0.1;
        } else if (xkeys.wristOffsetUp()) {
            escalator.wristManualOffset += 0.1;
        }

        if (Controls.cubeIntake()) {
            globalState.gamepiece = GamePiece.CUBE;
        }
        if (Controls.coneIntake()) {
            globalState.gamepiece = GamePiece.CONE;
        }

        if (!swerve.DISABLED.isActive()) {
            if (Controls.resetEncoders()) {
                swerve.resetEncoders();
            }
            if (Controls.zeroHeading()) {
                swerve.zeroHeading();
            }
            if (Controls.zeroOdom()) {
                swerve.zeroOdometry();
            }

            if (Controls.sttbolls() && !escalator.EEMOTIONAL.isActive()) {
                if (globalState.gamepiece == GamePiece.CONE) {
                    LL.TOP.LEDon();
                    LL.BOTTOM.LEDoff();
                } else {
                    LL.TOP.LEDon();
                    LL.BOTTOM.LEDon();
                }
                swerve.setCurrentState(swerve.STTBOLLS_AUTO);
                // if(swerve.con.readyToScore(swerve.getPose())) {
                // score();
                // swerve.setCurrentState(swerve.FIELD_RELATIVE_ABSOLUTE);
                // swerve.fromController(0, 0, 0); // only for one frame, just to be safe
                // }
            } else if (Controls.snap180()) {
                LL.TOP.LEDoff();
                LL.BOTTOM.LEDoff();
                swerve.setCurrentState(Controls.snap180Away() && Controls.snap180Toward() ? swerve.SNAP_HP
                        : Controls.snap180Away() ? swerve.SNAP_AWAY : swerve.SNAP_TOWARD);
                swerve.fromController(
                        -OI.precise(oi.getYLeftDriver()),
                        -OI.precise(oi.getXLeftDriver()),
                        0);
            } else if (xkeys.score()) {
                swerve.setCurrentState(swerve.BASE_LOCK_PASSIVE);
            } else {
                LL.TOP.LEDoff();
                LL.BOTTOM.LEDoff();
                boolean slow = Controls.throttle();
                double scale = slow ? 0.8 : 1;
                swerve.setCurrentState(swerve.FIELD_RELATIVE_CLASSIC);
                swerve.fromController(
                        -OI.precise(oi.getYLeftDriver()) * scale,
                        -OI.precise(oi.getXLeftDriver()) * scale,
                        -OI.precise(oi.getXRightDriver()) * scale);
            }
        }

        boolean in = Controls.intake();
        if (Controls.snap180Away() && Controls.snap180Toward()) {
            globalState.gamepiece = GamePiece.CONE;
            escalator.intakePos = IntakePos.HP;
            in = true;
        }
        if (Controls.coneIntake()) {
            globalState.gamepiece = GamePiece.CONE;
            escalator.intakePos = IntakePos.CONE;
        }
        if (Controls.cubeIntake()) {
            globalState.gamepiece = GamePiece.CUBE;
            escalator.intakePos = IntakePos.CUBE;
        }
        if (Controls.intakeNoSpin()) {
            globalState.gamepiece = GamePiece.CONE;
            escalator.intakePos = IntakePos.NO_SPIN;
        }

        if (in) {
            LL.TOP.setPipeline(3);
            LL.TOP.CAMdriver();
        } else {
            LL.TOP.CAMvision();
            LL.TOP.setPipeline(LL.pipeline(globalState.gamepiece, globalState.scoringLevel));
        }

        manualSuperstructure = manualSuperstructure || xkeys.stopSuperstructure();

        if (manualSuperstructure) {

            escalator.setCurrentState(escalator.MANUAL);
            escalator.manual(xkeys.manualEscalator() * 0.15, xkeys.manualLinx() * 0.18, xkeys.manualWrist() * 90);

            if (xkeys.zeroSuperstructure()) {
                manualSuperstructure = false;
                escalator.setCurrentState(escalator.ZEROING);
            }

        } else if (!escalator.DISABLED.isActive()) {
            // if (Controls.zeroWrist()) {
            // escalator.zeroWristStowed();
            // }
            if (Controls.zeroSuperstructure() || xkeys.zeroSuperstructure()) {
                escalator.setCurrentState(escalator.ZEROING);
            }

            if (Controls.score()) {
                score();
            } else if (escalator.INTAKING.isActive() && !in) {
                escalator.setCurrentState(escalator.STOWING);
            } else if (Controls.fire()) {
                shoot();
            } else if (escalator.EEMOTIONAL.isActive()) {
                // cancel logic
            } else if (in
                    && (escalator.ZEROING.isActive() || escalator.STOWED.isActive() || escalator.STOWING.isActive())) {
                escalator.setCurrentState(escalator.INTAKING);
            } else if (xkeys.iWasWrong()) {
                emotion(escalator.coneFree, escalator.ZEROING);
            }
        }

        if (!intake.DISABLED.isActive()) {
            if (escalator.EEMOTIONAL.isActive()) {
                intake.setCurrentState(intake.EEMOTIONAL);
            } else if (in && !Controls.intakeNoSpin()) {
                intake.setCurrentState(globalState.gamepiece == GamePiece.CONE ? intake.INTAKECONE : intake.INTAKECUBE);
            } else if (Controls.intakeNoSpin()) {
                intake.setCurrentState(intake.IDLE);
            } else if (Controls.spit()) {
                intake.setCurrentState(
                        globalState.gamepiece == GamePiece.CONE ? intake.RELEASECONE : intake.RELEASECUBE);
            } else if (!intake.EEMOTIONAL.isActive()) {
                intake.setCurrentState(intake.HOLD);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
        swerve.setCurrentState(swerve.DISABLED);
        escalator.setCurrentState(escalator.DISABLED);
        intake.setCurrentState(intake.DISABLED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
