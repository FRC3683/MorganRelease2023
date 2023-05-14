// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.config.Constants;
import frc.robot.utils.DavePath;
import frc.robot.utils.LL;

/** Add your docs here. */
public class SSGShaqGOATBlue extends DavePath {
    final double s80 = Constants.swerveTrueMaxSpeed_mps * 1.0;
    final double si = Constants.swerveTrueMaxSpeed_mps * 0.4;
    final double acc = 3.3;
    final double accSlow = 1.6;
    double i = -1;
    double cubeOffset = 0.69 - 0.1;

    boolean balance;
    Debouncer debouncer;

    public SSGShaqGOATBlue(boolean balance) {
        super(true);
        this.balance = balance;

        debouncer = new Debouncer(0.1);
        try {
            addWaitSegment(escalator.coneMid.totalTime() * 0.525);
            addHolonomicSegment(s80, acc*0.8, 7.0, 10, 0, si,
            i*180, i*0.2,
                    1.8, i*3.0, i*0.0, 1.0,
                    3.2, i*3.3, i*0.0, 1.0,
                    5.5, i*3.625, i*0.0, 1.0); // 180
            appendHolonomicSegment(si, acc, 12.0, 20,
                    0.0, i*0,
                    7.0, i*3.875, i*0.0, 1.0); // up to cube
            appendHolonomicSegment(s80, acc, 12.0, 20,
                    s80, i*135,
                    5.5, i*3.3, i*180.0, 1.0); // 180
            appendStrafeSegment(s80, accSlow, 12.0, 20,
                    0.0, i*180,
                    1.65, i*4.1, i*180.0, 1.0); // up to LL
            appendHolonomicSegment(s80, acc, 12.0, 20,
                    0, i*45,
                    5.0, i*3.47, i*0.0, 1.0,
                    7.18, i*5.315, i*45.0, 2.0); // to 2nd cube

            if (balance) {
                appendHolonomicSegment(s80 * 0.5, acc, 12.0, 20,
                        0.8, i*180,
                        4.55, i*5.7, i*180.0, 1.0); // 180
                addBalanceSegment();
            } else {
                appendStrafeSegment(s80, accSlow, 12.0, 20,
                        0, i*180,
                        5.4, i*4.05, i*180.0, 1.0,
                        1.45, i*4.45, i*180, 2.0); // to LL
                addWaitSegment(escalator.ballMid.totalTime() * 0.15);
                appendHolonomicSegment(s80, acc, 12, 20, 0, i*180, 
                        7.7, i*3.6, i*0, 1.0); // leave
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
            emote(escalator.ball420);
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