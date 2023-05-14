package frc.robot.commands;

import java.util.PriorityQueue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Controls;
import frc.robot.subsystems.EEscalator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.GlobalState;
import frc.robot.utils.LL;
import frc.robot.utils.MathUtils;
import frc.robot.utils.OI;
import frc.robot.utils.Xkey60;

public class EEscalatorBringup extends CommandBase {

    private static Xkey60 xkeys;
    private final GlobalState globalState;
    private final Swerve swerve;
    private final EEscalator escalator;
    private final Intake intake;
    private final OI oi;

    public EEscalatorBringup() {
        escalator = EEscalator.getInstance();
        intake = Intake.getInstance();
        swerve = Swerve.getInstance();
        xkeys = Xkey60.getInstance();
        oi = OI.getInstance();
        globalState = GlobalState.getInstance();
        addRequirements(escalator, intake, swerve);
    }

    @Override
    public void initialize() {
        swerve.setCurrentState(swerve.DISABLED);
        escalator.setCurrentState(escalator.ZEROING);
        intake.setCurrentState(intake.DISABLED);
        escalator.zeroWristAbs();
        escalator.zeroEscalator();
        escalator.zeroLinx();

    }

    @Override
    public void execute() {
        if (oi.getDPadDownDriver()) {
            escalator.setCurrentState(escalator.ZEROING);
        }

        if (oi.getAButtonDriver()) {
            escalator.setCurrentState(escalator.HOLDING);
        } else if (oi.getXButtonDriver()) {
            escalator.setCurrentState(escalator.TEST_EXTENDING);
        } else if (!escalator.ZEROING.isActive() && !escalator.TEST_EXTENDING.isActive()
                && !escalator.HOLDING.isActive()) {
            escalator.setCurrentState(escalator.IDLE);
        }

    }

    @Override
    public void end(boolean interrupted) {
        swerve.setCurrentState(swerve.DISABLED);
        intake.setCurrentState(intake.DISABLED);
        escalator.setCurrentState(escalator.DISABLED);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
