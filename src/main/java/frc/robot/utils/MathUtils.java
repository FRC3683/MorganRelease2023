package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Add your docs here.
 */
public class MathUtils {
    public static double sq(double x) {
        return x * x;
    }

    public static double average(double a, double b) {
        return (a + b) / 2;
    }

    public static double cube(double x) {
        return x * x * x;
    }

    public static double qu(double x) {
        return x * x * x * x;
    }

    public static double lerp(double start, double end, double alpha) {
        return start * (1.0 - alpha) + end * alpha;
    }

    public static double unlerp(double start, double end, double value) {
        return (value - start) / (end - start);
    }

    public static double remap(double old_start, double old_end, double old_value, double new_start, double new_end) {
        return lerp(new_start, new_end, unlerp(old_start, old_end, old_value));
    }

    public static double cubicBezier(double start, double start_q, double end, double end_q, double alpha) {
        double a = start * cube(1.0 - alpha);
        double b = 3 * start_q * sq(1.0 - alpha) * alpha;
        double c = 3 * end_q * (1.0 - alpha) * sq(alpha);
        double d = end * cube(alpha);
        return a + b + c + d;
    }

    public static double clamp(double min, double max, double value) {
        return value < min ? min : (value > max ? max : value);
    }
    public static int clampi(int min, int max, int value) {
        return value < min ? min : (value > max ? max : value);
    }

    public static double clamp01(double value) {
        return clamp(0.0, 1.0, value);
    }

    public static double clamp1(double value) {
        return clamp(-1.0, 1.0, value);
    }

    public static double clampN(double n, double value) {
        return clamp(-n, n, value);
    }

    public static double clamp0N(double max, double value) {
        return clamp(0.0, max, value);
    }

    public static double squaredInput(double x) {
        return x > 0 ? sq(x) : -sq(x);
    }

    public static double calcLeftDrive(double thrust, double rotate) {
        return -thrust - rotate;
    }

    public static double calcRightDrive(double thrust, double rotate) {
        return -thrust + rotate;
    }

    public static double calcLeftDriveDynamic(double thrust, double rotate) {
        double result = 0.0;
        if (thrust > 0.0) {
            if (rotate > 0.0f) {
                result = -sq(thrust) - sq(rotate);
            } else {
                result = -sq(thrust) + sq(rotate);
            }
        } else {
            if (rotate > 0.0f) {
                result = sq(thrust) - sq(rotate);
            } else {
                result = sq(thrust) + sq(rotate);
            }
        }
        return result;
    }

    public static double calcRightDriveDynamic(double thrust, double rotate) {
        double result = 0.0;
        if (thrust > 0.0) {
            if (rotate > 0.0f) {
                result = -sq(thrust) + sq(rotate);
            } else {
                result = -sq(thrust) - sq(rotate);
            }
        } else {
            if (rotate > 0.0f) {
                result = sq(thrust) + sq(rotate);
            } else {
                result = sq(thrust) - sq(rotate);
            }
        }
        return result;
    }

    public static boolean closeEnough(double num1, double num2, double error) {
        return Math.abs(num1 - num2) <= error;
    }

    public static boolean closeEnoughPercent(double num1, double num2, double alpha) {
        return Math.abs(num1 - num2) <= Math.abs(num2 * alpha);
    }

    private static double[] EMPTY_DOUBLE_ARRAY = {};

    public static double[] toPrimitive(Double[] array) {
        if (array == null) {
            return null;
        } else if (array.length == 0) {
            return EMPTY_DOUBLE_ARRAY;
        }
        final double[] result = new double[array.length];
        for (int i = 0; i < array.length; i++) {
            result[i] = array[i].doubleValue();
        }
        return result;
    }

    public static DaveSwerveModuleState2 swerveModuleState(double degrees, double mps) {
        DaveSwerveModuleState2 sms = new DaveSwerveModuleState2();
        sms.angle = Rotation2d.fromDegrees(degrees);
        sms.speedMetersPerSecond = mps;
        sms.omegaRadPerSecond = 0.0;
        return sms;
    }

    public static boolean withinCircle(double dx, double dy, double r) {
        return (dx * dx + dy * dy) < (r * r);
    }

    /**
     * Grand theft poofs code
     * Obtain a new Pose2d from a (constant curvature) velocity. See:
     * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
     */
    public static Pose2d exp(final Twist2d delta) {
        double sin_theta = Math.sin(delta.dtheta);
        double cos_theta = Math.cos(delta.dtheta);
        double s, c;
        if (Math.abs(delta.dtheta) < 1E-9) {
            s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
            c = 0.5 * delta.dtheta;
        } else {
            s = sin_theta / delta.dtheta;
            c = (1.0 - cos_theta) / delta.dtheta;
        }
        return new Pose2d(new Translation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
                new Rotation2d(cos_theta, sin_theta));
    }

    /**
     * Logical inverse of the above.
     */
    public static Twist2d log(final Pose2d transform) {
        final double dtheta = transform.getRotation().getRadians();
        final double half_dtheta = 0.5 * dtheta;
        final double cos_minus_one = transform.getRotation().getCos() - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < 1E-9) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().getSin()) / cos_minus_one;
        }
        final Translation2d translation_part = transform.getTranslation()
                .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
        return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
    }

    public static double closest90deg(double deg) {
        return Math.round(deg / 90.0) * 90.0;
    }

    public static double closest90rad(double rad) {
        return Math.round(rad / (Math.PI / 2)) * (Math.PI / 2);
    }
}