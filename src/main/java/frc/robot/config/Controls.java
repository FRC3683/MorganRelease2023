package frc.robot.config;

import frc.robot.utils.OI;
import frc.robot.utils.Xkey60;

public class Controls {
    private static OI oi = OI.getInstance();

    public static boolean zeroHeading() {
        return oi.getStartButtonDriver();
    }
    public static boolean zeroWrist() {
        return oi.getBackButtonDriver();
    }
    public static boolean resetEncoders() {
        return oi.getDPadDownDriver();
    }
    public static boolean zeroOdom() {
        return oi.getDPadUpDriver();
    }
    public static boolean zeroSuperstructure() {
        return oi.getDPadLeftDriver();
    }

    public static boolean sttbolls() {
        return oi.getLeftBumperDriver();
    }
    public static boolean snap180Toward() {
        return oi.getYButtonDriver();
    }
    public static boolean snap180Away() {
        return oi.getAButtonDriver();
    }
    public static boolean snap180() {
        return snap180Away() || snap180Toward();
    }
    public static boolean score() {
        return oi.getXButtonDriver();
    }

    public static boolean coneIntake() {
        return oi.getRightTriggerDriver() > 0.5 && oi.getLeftTriggerDriver() < 0.5;
    }
    public static boolean cubeIntake() {
        return oi.getLeftTriggerDriver() > 0.5 && oi.getRightTriggerDriver() < 0.5;
    }
    public static boolean intakeNoSpin() {
        return oi.getRightTriggerDriver() > 0.5 && oi.getLeftTriggerDriver() > 0.5;
    }
    public static boolean intake() {
        return intakeNoSpin() || coneIntake() || cubeIntake();
    }

    public static boolean spit() {
        return oi.getRightBumperDriver();
    }

    public static boolean fire() {
        return oi.getBButtonDriver();
    }

    public static boolean throttle() {
        return intake();
        // return oi.getLeftBumperDriver();
    }

    public static boolean eescalatorCoast() {
        return false;
    }
}
