// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.config.Constants;
import frc.robot.utils.DavePath;
import frc.robot.utils.LL;

/** Add your docs here. */
public class SSGKob3Blue extends DavePath {

    final double s80 = Constants.swerveTrueMaxSpeed_mps * 1.0;
    final double sBmp = Constants.swerveTrueMaxSpeed_mps * 0.25;
    final double acc = 3.5;
    final double i = -1;
    boolean balance;
    Debouncer balance_debouncer;

    public SSGKob3Blue(boolean balance) {
        super(true);
        this.balance = balance;

        balance_debouncer = new Debouncer(0.1);
        try {
            addWaitSegment(escalator.coneMid.totalTime() * 0.525);
            addHolonomicSegment(s80, acc, 12.0, 20.0, 0, sBmp, i*180, i*-90,
                    1.8, i*7.5, i*-40.0, 1.0,
                    2.3, i*7.38, i*0.0, 1.0,
                    3.2, i*7.38, i*0.0, 1.0); // up to bump
            appendHolonomicSegment(sBmp, acc, 12.0, 20.0, sBmp, i*-90,
                    4.75, i*7.3, i*0.0, 1.0); // across cable bump
            appendHolonomicSegment(s80, acc, 12.0, 20.0, s80, i*0,
                    5.58, i*6.81, i*0.0, 1.0); // up to cube
            appendHolonomicSegment(s80, acc, 12.0, 20.0, 0, i*-1,
                    7.16, i*6.85, i*0.0, 1.0); // intake cube
            appendHolonomicSegment(s80, acc, 12.0, 20.0,
                    0, i*-180,
                    5.05, i*6.70, i*180.0, 1.0); // up to cable bump
            addWaitSegment(escalator.ballShootHigh.totalTime() - 0.9);
            appendHolonomicSegment(s80, acc, 12.0, 20.0,
                    0, i*-45,
                    5.75, i*6.9, i*0.0, 1.0,
                    7.525, i*5.135, i*-20.0, 1.0); // up to cube
            if (balance) {

                appendStrafeSegment(s80 * 0.5, acc, 12, 20, 0.0, i*180,
                        2.98, i*4.3, i*180, 1.0);
                addBalanceSegment();
            } else {
                appendHolonomicSegment(s80, acc, 12.0, 20.0,
                        sBmp, i*180,
                        4.6, i*6.65, i*180.0, 1.0);// up to cable bump
                appendHolonomicSegment(sBmp * 2.5, acc, 12.0, 20.0, sBmp, i*180,
                        3.4, i*6.75, i*180.0, 1.0); // across cable bump
                appendStrafeSegment(s80, acc*0.7, 12.0, 20.0,
                        0.0, i*180,
                        1.15, i*6.65, i*180, 1.0); // Score
                addWaitSegment(escalator.ballMid.totalTime() * 0.2);
                appendHolonomicSegment(s80*0.5, acc, 12, 20, 0, i*180,
                        7.7, i*7, i*0, 1.0);
            }

            addWaitSegment(3.0);

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void onStart() {
        if (balance) {
            LL.TOP.setPipeline(4);
        } else {
            LL.TOP.setPipeline(2);
        }

        LL.BOTTOM.LEDoff();
        escalator.setCurrentState(escalator.ZEROING);
        swerve.setCurrentState(swerve.IDLE);
        // swerve.setHeading(180);
    }

    @Override
    public void onExecute() {
        if (before(segEnd(0))) {
            emote(escalator.coneMid);
        } else if (between(segEnd(3), segEnd(4))) {
            // if(intake.hasBall()){
            // stow();
            // } else {
            intakeCube();
            // }
        } else if (between(segEnd(4), segEnd(5))) {
            stow();
        }
        if (between(segEnd(5) - 0.3, segEnd(5))) {
            emote(escalator.ballShootHigh);
        } else if (between(segEnd(6) + 1.0, segEnd(7))) {
            // if(intake.hasBall()){
            // stow();
            // } else {
            intakeCube();
            // }
        } else if (between(segEnd(7), segEnd(8) - 0.5)) {
            stow();
        }

        if (between(segEnd(7) + 0.3, segEnd(8) + 1.0) && !escalator.EEMOTIONAL.isActive()) {
            stow();
        }

        if (!balance) {
            if (between(segEnd(9) - 0.5, segEnd(10))) {
                cubeLLNear();
            }

            if (between(segEnd(10) - escalator.ballMid.totalTime() * 0.4, segEnd(10))) {
                emote(escalator.ballMid);
            }
        } else {
            if (between(segEnd(6) - 0.5, segEnd(6))) {
                cubeLLFar();
            }
            if (swerve.BALANCING.isActive() && balance_debouncer.calculate(swerve.con.isNaiveBalanaced())
                    && before(15.0 - escalator.ballShootHigh.totalTime())) {
                swerve.setCurrentState(swerve.BASE_LOCK_PASSIVE);
                if (LL.TOP.validTarget() && swerve.con.distanceCS() > 2.54)
                    emote(escalator.ballShootMid);
                else
                    emote(escalator.ballShootHigh);
            }
        }
    }

}
