// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.utils.GlobalState.ScoringLevel;
import frc.robot.utils.GlobalState.ScoringOffset;
import frc.robot.utils.GlobalState.ScoringSlot;

/** Add your docs here. */
public class Xkey60 extends GenericHID {
    private static Xkey60 instance;

    public static Xkey60 getInstance() {
        if (instance == null) {
            instance = new Xkey60(1);
        }
        return instance;
    }

    private Xkey60(final int port) {
        super(port);
        HAL.report(tResourceType.kResourceType_XboxController, port + 1);
    }

    public ScoringSlot scoringSlot() {
        int degrees = getPOV();
        if (degrees == -1) {
            return ScoringSlot.NINE;
        }
        return ScoringSlot.values()[degrees / 45];
    }

    public double manualEscalator() {
        return getRawAxis(0);
    }

    public double manualLinx() {
        return getRawAxis(1);
    }

    public double manualWrist() {
        return getRawAxis(3);
    }

    public boolean flashCone() {
        return getRawButton(1);
    }

    public boolean flashCube() {
        return getRawButton(2);
    }

    public boolean flashBetterThanQuicksilver() {
        return getRawButton(16);
    }

    private int scoringBits() {
        int res = 0;
        res |= getRawButton(6) ? 1 << 3 : 0;
        res |= getRawButton(5) ? 1 << 2 : 0;
        res |= getRawButton(4) ? 1 << 1 : 0;
        res |= getRawButton(3) ? 1 << 0 : 0;
        return res;
    }

    public ScoringLevel scoringLevel() {
        final ScoringLevel[] levels = { ScoringLevel.HIGH, ScoringLevel.MID, ScoringLevel.LOW };
        return levels[scoringBits() % 3];
    }

    public ScoringOffset scoringOffset() {
        switch (scoringSlot()) {
            case FOUR:
                return ScoringOffset.HEAVY_LEFT;
            case ONE:
                return ScoringOffset.LEFT;
            case TWO:
                return ScoringOffset.CENTER;
            case THREE:
                return ScoringOffset.RIGHT;
            case SIX:
                return ScoringOffset.HEAVY_RIGHT;
            default:
                break;
        }
        return ScoringOffset.values()[scoringBits() / 3];
    }

    public boolean disableConeBeam() {
        return getRawButton(7);
    }

    public boolean disableCubeBeam() {
        return getRawButton(8);
    }

    public boolean score() {
        return getRawButton(9);
    }

    public boolean stopSuperstructure() {
        return getRawButton(10);
    }

    public boolean zeroSuperstructure() {
        return getRawButton(11);
    }

    public boolean autoGenerate() {
        return getRawButtonPressed(13);
    }

    public boolean autoCycle() {
        return getRawButtonPressed(12);
    }

    public boolean wristOffsetUp() {
        return getRawButtonPressed(14);
    }

    public boolean wristOffsetDown() {
        return getRawButtonPressed(15);
    }

    public boolean iWasWrong() {
        return getRawButton(16);
    }
}
