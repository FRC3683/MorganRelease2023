package frc.robot.config;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AutoConstants {

    public static final double kMaxSpeedMetersPerSecond = 2.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4;
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 3.0;
    private static final double kMaxAngularRadiansPerSecond = Math.PI / 2;
    private static final double kMaxAngularAccellerationRadiansPerSecond = 3;
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularRadiansPerSecond,
            kMaxAngularAccellerationRadiansPerSecond);

}
