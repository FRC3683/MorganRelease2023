package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class OI {

  XboxController driver;
  XboxController operator;
  private Timer driverTimer;
  private Timer operatorTimer;
  private static OI instance;

  private double driverRumbleTime;
  private double operatorRumbleTime;

  private OI() {
    driver = new XboxController(0);
    operator = new XboxController(1);
    driverTimer = new Timer();
    operatorTimer = new Timer();
    driverRumbleTime = -1.0;
    operatorRumbleTime = -1.0;
    driverTimer.start();
    operatorTimer.start();
  }

  public static OI getInstance() {
    if (instance == null) {
      instance = new OI();
    }
    return instance;
  }

  private static final double DEADBAND_RADIUS = 0.10;

  private static double deadband(double jsValue) {
    double res = 0.0;

    if (jsValue > DEADBAND_RADIUS) {
      res = MathUtils.lerp(DEADBAND_RADIUS, 1.0, jsValue);
    } else if (jsValue < -DEADBAND_RADIUS) {
      res = MathUtils.lerp(-DEADBAND_RADIUS, -1.0, -jsValue);
    }

    return res;
  }

  public double getXLeftDriver() {
    return deadband(driver.getLeftX());
  }

  public double getXLeftOperator() {
    return deadband(operator.getLeftX());
  }

  public double getYLeftDriver() {
    return deadband(driver.getLeftY());
  }

  public double getYLeftOperator() {
    return deadband(operator.getLeftY());
  }

  public double getXRightDriver() {
    return deadband(driver.getRightX());
  }

  public double getXRightOperator() {
    return deadband(operator.getRightX());
  }

  public double getYRightDriver() {
    return deadband(driver.getRightY());
  }

  public double getYRightOperator() {
    return deadband(operator.getRightY());
  }

  public boolean getAButtonDriver() {
    return driver.getAButton();
  }

  public boolean getAButtonOperator() {
    return operator.getAButton();
  }

  public boolean getBButtonDriver() {
    return driver.getBButton();
  }

  public boolean getBButtonOperator() {
    return operator.getBButton();
  }

  public boolean getXButtonDriver() {
    return driver.getXButton();
  }

  public boolean getXButtonOperator() {
    return operator.getXButton();
  }

  public boolean getYButtonDriver() {
    return driver.getYButton();
  }

  public boolean getYButtonOperator() {
    return driver.getYButton();
  }

  public boolean getLeftBumperDriver() {
    return driver.getLeftBumper();
  }

  public boolean getLeftBumperOperator() {
    return operator.getLeftBumper();
  }

  public boolean getRightBumperDriver() {
    return driver.getRightBumper();
  }

  public boolean getRightBumperOperator() {
    return operator.getRightBumper();
  }

  public boolean getBackButtonDriver() {
    return driver.getBackButton();
  }

  public boolean getBackButtonOperator() {
    return operator.getBackButton();
  }

  public boolean getStartButtonDriver() {
    return driver.getStartButton();
  }

  public boolean getStartButtonOperator() {
    return operator.getStartButton();
  }

  public boolean getLeftStickButtonDriver() {
    return driver.getLeftStickButton();
  }

  public boolean getLeftStickButtonOperator() {
    return operator.getLeftStickButton();
  }

  public boolean getRightStickButtonDriver() {
    return driver.getRightStickButton();
  }

  public boolean getRightStickButtonOperator() {
    return operator.getRightStickButton();
  }

  public double getLeftTriggerDriver() {
    return driver.getLeftTriggerAxis();
  }

  public double getLeftTriggerOperator() {
    return operator.getLeftTriggerAxis();
  }

  public double getRightTriggerDriver() {
    return driver.getRightTriggerAxis();
  }

  public double getRightTriggerOperator() {
    return operator.getRightTriggerAxis();
  }

  public boolean getDPadLeftDriver() {
    return driver.getPOV() == 270;
  }

  public boolean getDPadLeftOperator() {
    return operator.getPOV() == 270;
  }

  public boolean getDPadRightDriver() {
    return driver.getPOV() == 90;
  }

  public boolean getDPadRightOperator() {
    return operator.getPOV() == 90;
  }

  public boolean getDPadUpDriver() {
    return driver.getPOV() == 0;
  }

  public boolean getDPadUpOperator() {
    return operator.getPOV() == 0;
  }

  public boolean getDPadDownDriver() {
    return driver.getPOV() == 180;
  }

  public boolean getDPadDownOperator() {
    return operator.getPOV() == 180;
  }

  public boolean getAButtonPressedDriver() {
    return (driver.getAButtonPressed());
  }

  public boolean getAButtonPressedOperator() {
    return (operator.getAButtonPressed());
  }

  public boolean getBButtonPressedDriver() {
    return (driver.getBButtonPressed());
  }

  public boolean getBButtonPressedOperator() {
    return (operator.getBButtonPressed());
  }

  public boolean getXButtonPressedDriver() {
    return (driver.getXButtonPressed());
  }

  public boolean getXButtonPressedOperator() {
    return (operator.getXButtonPressed());
  }

  public boolean getYButtonPressedDriver() {
    return (driver.getYButtonPressed());
  }

  public boolean getYButtonPressedOperator() {
    return (operator.getYButtonPressed());
  }

  public boolean getLeftBumperPressedDriver() {
    return (driver.getLeftBumperPressed());
  }

  public boolean getLeftBumperPressedOperator() {
    return (operator.getLeftBumperPressed());
  }

  public boolean getRightBumperPressedDriver() {
    return (driver.getRightBumperPressed());
  }

  public boolean getRightBumperPressedOperator() {
    return (operator.getRightBumperPressed());
  }

  public boolean getBackButtonPressedDriver() {
    return (driver.getBackButtonPressed());
  }

  public boolean getBackButtonPressedOperator() {
    return (operator.getBackButtonPressed());
  }

  public boolean getStartButtonPressedDriver() {
    return (driver.getStartButtonPressed());
  }

  public boolean getStartButtonPressedOperator() {
    return (operator.getStartButtonPressed());
  }

  public boolean getLeftStickButtonPressedDriver() {
    return (driver.getLeftStickButtonPressed());
  }

  public boolean getLeftStickButtonPressedOperator() {
    return (operator.getLeftStickButtonPressed());
  }

  public boolean getRightStickButtonPressedDriver() {
    return (driver.getRightStickButtonPressed());
  }

  public boolean getRightStickButtonPressedOperator() {
    return (operator.getRightStickButtonPressed());
  }

  public void rumbleDriver(double lpower, double rpower) {
    driver.setRumble(Joystick.RumbleType.kLeftRumble, lpower);
    driver.setRumble(Joystick.RumbleType.kRightRumble, rpower);
  }

  public void rumbleOperator(double lpower, double rpower) {
    operator.setRumble(Joystick.RumbleType.kLeftRumble, lpower);
    operator.setRumble(Joystick.RumbleType.kRightRumble, rpower);
  }

  public void rumbleDriverFor(double lpower, double rpower, double time) {
    driver.setRumble(Joystick.RumbleType.kLeftRumble, lpower);
    driver.setRumble(Joystick.RumbleType.kRightRumble, rpower);
    driverRumbleTime = time;
    driverTimer.reset();
  }

  public void rumbleOperatorFor(double lpower, double rpower, double time) {
    operator.setRumble(Joystick.RumbleType.kLeftRumble, lpower);
    operator.setRumble(Joystick.RumbleType.kRightRumble, rpower);
    operatorRumbleTime = time;
    operatorTimer.reset();
  }

  public void update() {
    if (driverTimer.get() > driverRumbleTime)
      rumbleDriver(0.0, 0.0);
    if (operatorTimer.get() > operatorRumbleTime)
      rumbleOperator(0.0, 0.0);
  }

  public boolean driverInput() {
    if (driver.getAButton())
      return true;
    if (driver.getBButton())
      return true;
    if (driver.getXButton())
      return true;
    if (driver.getYButton())
      return true;
    if (driver.getStartButton())
      return true;
    if (driver.getBackButton())
      return true;
    if (driver.getPOV() != -1)
      return true;
    if (driver.getLeftStickButton())
      return true;
    if (driver.getRightStickButton())
      return true;
    if (driver.getLeftBumper())
      return true;
    if (driver.getRightBumper())
      return true;
    if (getLeftTriggerDriver() > 0.0)
      return true;
    if (getRightTriggerDriver() > 0.0)
      return true;
    if (getYLeftDriver() != 0.0)
      return true;
    if (getXLeftDriver() != 0.0)
      return true;
    if (getYRightDriver() != 0.0)
      return true;
    if (getXRightDriver() != 0.0)
      return true;
    return false;
  }

  public boolean operatorInput() {
    if (operator.getAButton())
      return true;
    if (operator.getBButton())
      return true;
    if (operator.getXButton())
      return true;
    if (operator.getYButton())
      return true;
    if (operator.getStartButton())
      return true;
    if (operator.getBackButton())
      return true;
    if (operator.getPOV() != -1)
      return true;
    if (operator.getLeftStickButton())
      return true;
    if (operator.getRightStickButton())
      return true;
    if (operator.getLeftBumper())
      return true;
    if (operator.getRightBumper())
      return true;
    if (getLeftTriggerOperator() > 0.0)
      return true;
    if (getRightTriggerOperator() > 0.0)
      return true;
    if (getYLeftOperator() != 0.0)
      return true;
    if (getXLeftOperator() != 0.0)
      return true;
    if (getYRightOperator() != 0.0)
      return true;
    if (getXRightOperator() != 0.0)
      return true;
    return false;
  }

  public static double precise(double input) {
    return Math.signum(input) * MathUtils.sq(input);
  }

}