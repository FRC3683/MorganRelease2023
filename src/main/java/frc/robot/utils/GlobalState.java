
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class GlobalState {

    public enum ScoringOffset {
        CENTER(0, 0),
        LEFT(0.04, 0.02),
        RIGHT(-0.04, -0.02),
        HEAVY_LEFT(0.08, 0.03),
        HEAVY_RIGHT(-0.08, -0.03);

        // DO NOT CHANGE ORDER IT MATTERS FOR XKEYS CONVERSION

        public double offset(GamePiece gamepiece) {
            return gamepiece == GamePiece.CUBE ? cubeOffset : coneOffset;
        }

        private final double coneOffset, cubeOffset;

        private ScoringOffset(double coneOffset, double cubeOffset) {
            this.coneOffset = coneOffset;
            this.cubeOffset = cubeOffset;
        }
    }

    public enum GamePiece {
        CONE(0),
        CUBE(1);

        public final int index;

        private GamePiece(int index) {
            this.index = index;
        }
    }

    public enum ScoringLevel {
        LOW(0),
        MID(1),
        HIGH(2);

        public final int index;

        private ScoringLevel(int index) {
            this.index = index;
        }
    }

    public enum ScoringSlot {
        ONE(0),
        TWO(1),
        THREE(2),
        FOUR(3),
        FIVE(4),
        SIX(5),
        SEVEN(6),
        EIGHT(7),
        NINE(8);

        // DO NOT CHANGE ORDER IT MATTERS FOR XKEYS CONVERSION

        public final int index;

        private ScoringSlot(int index) {
            this.index = index;
        }
    }

    private Alliance alliance;
    public GamePiece gamepiece;
    public ScoringLevel scoringLevel;
    public ScoringSlot scoringSlot;
    public ScoringOffset scoringOffset;
    public boolean disableConeBeam = false;
    public boolean disableBallBeam = false;

    private GlobalState() {
        alliance = Alliance.Invalid;
        gamepiece = GamePiece.CONE;
        scoringLevel = ScoringLevel.HIGH;
        scoringSlot = ScoringSlot.ONE;
        scoringOffset = ScoringOffset.CENTER;
    }

    public void pollFMS() {
        alliance = DriverStation.getAlliance();
    }

    public GamePiece getGamePieceFromScoringLocation() {
        if (scoringLevel == ScoringLevel.HIGH || scoringLevel == ScoringLevel.MID) {
            if (scoringSlot == ScoringSlot.TWO || scoringSlot == ScoringSlot.FIVE || scoringSlot == ScoringSlot.EIGHT) {
                return GamePiece.CUBE;
            }
            return GamePiece.CONE;
        }
        return GamePiece.CONE;
    }

    public Alliance alliance() {
        return alliance;
    }

    private static GlobalState inst;

    public static GlobalState getInstance() {
        if (inst == null) {
            inst = new GlobalState();
        }
        return inst;
    }
}
