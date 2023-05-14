// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.config.Constants;
import frc.robot.utils.DavePath;
import frc.robot.utils.LL;

/** Add your docs here. */
public class SSGShaqPracticeRed extends DavePath {
    final double s80 = Constants.swerveTrueMaxSpeed_mps * 1.0;
    final double si = Constants.swerveTrueMaxSpeed_mps * 0.4;
    final double acc = 3.2;
    final double accSlow = 2.2;
    double cubeOffset = 0.69 - 0.1;

    boolean balance;
    Debouncer debouncer;

    public SSGShaqPracticeRed(boolean balance) {
        super(true);
        this.balance = balance;

        debouncer = new Debouncer(0.1);
        try {
            addWaitSegment(escalator.coneMid.totalTime() * 0.525);
            addHolonomicSegment(s80, acc * 0.8, 7.0, 10, 0, si,
                    180, 0.2,
                    1.8, 3.0, 0.0, 1.0,
                    3.2, 3.1, 0.0, 1.0,
                    5.5, 3.525, 0.0, 1.0); // 180
            appendHolonomicSegment(si, acc, 12.0, 20,
                    0.0, 0,
                    7.0, 3.525, 0.0, 1.0); // up to cube
            appendHolonomicSegment(s80, 2.5, 12.0, 20,
                    s80, 135,
                    5.5, 3.1, 180.0, 1.0); // 180
            appendStrafeSegment(s80, accSlow, 12.0, 20,
                    0.0, 180,
                    1.8, 3.7, 180.0, 1.0); // up to LL
            appendHolonomicSegment(s80, acc, 12.0, 20,
                    0, 45,
                    5.0, 3.27, 0.0, 1.0,
                    7.38, 4.905, 45.0, 2.0); // to 2nd cube

            if (balance) {
                appendHolonomicSegment(s80 * 0.5, acc, 12.0, 20,
                        0.8, 180,
                        4.55, 5.3, 180.0, 1.0); // 180
                addBalanceSegment();
            } else {
                appendStrafeSegment(s80, acc, 12.0, 20,
                        0, 180,
                        6.0, 3.2, 180.0, 1.0,
                        1.7, 4.0, 180, 2.0); // to LL
                appendHolonomicSegment(s80, acc, 12.0, 20, 0, 180,
                        7.5, 3.8, 180, 1.0);
                // addWaitSegment(escalator.ballShootMid.totalTime());
            }

            addWaitSegment(3.0);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void onStart() {
        LL.TOP.setPipeline(2);

        LL.BOTTOM.LEDoff();
        escalator.setCurrentState(escalator.ZEROING);
        swerve.setCurrentState(swerve.IDLE);
        // swerve.setHeading(180);
    }

    @Override
    public void onExecute() {
        if (before(segEnd(0))) {
            emote(escalator.coneMid);
        } else if (between(segEnd(1), segEnd(2))) {
            cubeLLNear();
            if (intake.hasBall()) {
                stow();
            } else {
                intakeCube();
            }
        } else if (between(segEnd(2), segEnd(3))) {
            stow();
        }

        if (between(segEnd(4) - escalator.ball420.totalTime() * cubeOffset, segEnd(4))) {
            emote(balance ? escalator.ballMid : escalator.ball420);
        } else if (between(segEnd(5) - 1.0, segEnd(5) + 0.5)) {
            if (intake.hasBall()) {
                stow();
            } else {
                intakeCube();
            }
        } else if (between(segEnd(5) + 0.5, segEnd(5) + 0.75)) {
            stow();
        }

        if (balance) {
            if (after(14.9)) {
                swerve.setCurrentState(swerve.BASE_LOCK_PASSIVE);
            }
        } else {
            if (between(segEnd(6) - escalator.ballMid.totalTime() * 0.5, segEnd(6))) {
                emote(escalator.ballMid);
            }
        }
    }

}