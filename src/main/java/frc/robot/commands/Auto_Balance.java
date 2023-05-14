package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.Constants;
import frc.robot.utils.DavePath;
import frc.robot.utils.GlobalState;
import frc.robot.utils.LL;
import frc.robot.utils.GlobalState.GamePiece;

public class Auto_Balance extends DavePath {

    Debouncer debouncer;

    public Auto_Balance(boolean away, boolean facing_away) {
        super(true);
        double facing = facing_away ? 0 : 180;
        double path_degs = away ? 0 : 180;
        double drive_pos = away ? 2.2 : -2.2;

        try {
            addHolonomicSegment(1.5, 2.5, Constants.swerveMaxTurn_radps, 10, 0, 1.5, facing, facing, 
            0, 0, path_degs, 1.0,
            drive_pos, 0, path_degs, 1.0);
        } catch (Exception e) {
            e.printStackTrace();
        }

        addBalanceSegment();
    }

    boolean has_shot;

    @Override
    public void onStart() {
        has_shot = false;
        debouncer = new Debouncer(0.5);
        GlobalState.getInstance().gamepiece = GamePiece.CUBE;
        intake.setCurrentState(intake.HOLD);
        LL.TOP.setPipeline(4);
    }

    @Override
    public void onExecute() {
        if(swerve.BALANCING.isActive() && debouncer.calculate(swerve.con.isNaiveBalanaced())) {
            swerve.setCurrentState(swerve.STRAFING);
        }
        if(swerve.STRAFING.isActive() && !has_shot && swerve.con.leftRightStrafeReady()){// && (15 - DriverStation.getMatchTime()) < (escalator.ballShootHigh.totalTime() * 0.8)) {
            has_shot = true;
            escalator.setEmotion(escalator.ballShootHigh, escalator.ZEROING);
            intake.setEEMotion(escalator.ballShootHigh);
        }
    }
}
