package frc.robot.config;

public final class Constants {
        public static final double controlDt_s = 0.01;

        public static final double swerveGearRatio = 1 / 6.12;
        public static final double swerveCircumference = 0.1016 * Math.PI;
        public static final double motorDistanceMeters = swerveGearRatio * swerveCircumference;
        public static final double swerveDistFudgePractice = 12.25 / 14.0;
        public static final double swerveDistFudgeComp = 1;

        public static final double swerveTrueMaxSpeed_mps = 4.35;
        public static final double swerveMaxSpeed_mps = swerveTrueMaxSpeed_mps;
        public static final double swerveMaxTurn_radps = 12;

        public static final double wheelBase_meters = 0.52705; // front back delta = left right delta (distance between
                                                               // them
        // center of wheels) (20.75 in nominal)

        public static final double trueWidth_m = 0.84455;
        public static final double trueLength_m = 0.85725;

        public static final double nominalVoltage = 12.0;

        public static final double conebeam_debounce = 0.0;
        public static final double ballbeam_debounce = 0.15;
        public static final double cone_intake = 0.9;
        public static final double ball_intake = -0.75;
        public static final double cone_spit = -0.3;
        public static final double ball_spit = 0.9; // manual spit
        public static final double ball_score_spit = 0.38; // scoring spit
        public static final double cone_hold = 0.1;
        public static final double ball_hold = -0.03;

        public static final double escalator_angle_scaling = 1.0 / Math.sin(Math.toRadians(52));

        public static final double escalatorStowed_m = 0.298; // ground to wrist pivot
        public static final double escalatorExtended_m = 1.246;
        public static final double escalatorIntakeBall_m = escalatorStowed_m;
        public static final double escalatorIntakeCone_m = escalatorStowed_m;
        public static final double escalatorPrescore_m = 0.3; // TODO
        public static final double linxStowed_m = 0.382; // 0.199;
        public static final double linxExtended_m = 0.608; // 0.199+0.232;
        public static final double linxIntakeBall_m = linxExtended_m;
        public static final double linxIntakeCone_m = linxExtended_m;
        public static final double linxPrescore_m = 0.1; // TODO
        public static final double wristStowed_deg = 75.0; // from horizontal
        public static final double wristExtended_deg = -25.0;
        public static final double wristIntakeBall_deg = wristExtended_deg;
        public static final double wristIntakeCone_deg = wristExtended_deg;
        public static final double wristPrescore_deg = 65; // TODO

        public static final double escMaxVel_mps = 2.1;
        public static final double lnxMaxVel_mps = 0.8;
        public static final double wstMaxVel_dps = 300;

        public static final double escMaxAcc_mps2 = 8.0;
        public static final double lnxMaxAcc_mps2 = 2.5;
        public static final double wstMaxAcc_dps2 = 1200;

        // Recalc gains
        public static final double esckG_V = 1.05;
        public static final double esckV_Vspm = 6.59;
        public static final double esckA_Vs2pm = 0.15;
        public static final double lnxkG_V = 0.00;
        public static final double lnxkV_Vspm = 3.84;
        public static final double lnxkA_Vs2pm = 0.15;
        public static final double wrskG_V = 1.02;
        public static final double wrskV_Vspdeg = 0.01;
        public static final double wrskA_Vs2pdeg = 0.00;

        /*
         * ESCALATOR STOWED:
         * how far wrist can go down when linx is stowed: 4deg
         * intermediary linx: 0.312m, wrist: -10deg
         * wrist can go all the way down when linx is at: 0.401m
         * 
         * WRIST ALL THE WAY DOWN
         * MIN ELEVATOR HEIGHT: 0.376
         * MIN LINX DIST: 0.214
         * 
         * zeroing WRIST angle before linx goes in -1.12
         */

        public static final double linxLimit = 0.495;
        public static final double wristLimit = 30;

        public static final double escalatorRangeRaw = 36.666316986083984;
        public static final double linxRangeRaw = 5.238088607788086;
        public static final double wristRangeRaw = 12.78575325012207;

        // TODO add rotations
        public static final double escalatorMetersToRotations = (escalatorExtended_m - escalatorStowed_m)
                        / escalatorRangeRaw;
        public static final double linxMetersToRotations = (linxExtended_m - linxStowed_m) / linxRangeRaw;
        public static final double wristDegreesToRotations = 360.0 / 45.5;// (wristStowed_deg - wristExended_deg) /
                                                                          // wristRangeRaw;

}
