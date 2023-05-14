// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.EEscalatorBringup;
import frc.robot.commands.RobotTeleop;
import frc.robot.config.Config;
import frc.robot.config.Constants;
import frc.robot.subsystems.EEscalator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.AutoSelector;
import frc.robot.utils.GlobalState;
import frc.robot.utils.GlobalTimer;
import frc.robot.utils.LL;
import frc.robot.utils.OI;
// import frc.robot.utils.StateLogger;
import frc.robot.utils.Xkey60;

public class Robot extends TimedRobot {
  Swerve swerve;
  Intake intake;
  EEscalator escalator;

  public class MasterControlLoop implements Runnable {

    public MasterControlLoop() {
      swerve = Swerve.getInstance();
      intake = Intake.getInstance();
      escalator = EEscalator.getInstance();
    }

    @Override
    public void run() {
      swerve.controlsPeriodic();
      escalator.controlsPeriodic();
      intake.controlsPeriodic();
    }
  }

  private Command teleop;
  private MasterControlLoop controlsPeriodic;
  private Timer swerveFixTimer;
  private GlobalTimer globalTimer;
  private GlobalState globalState;
  private Xkey60 xkeys;
  private AutoSelector autoSelector;
  // private StateLogger logger;
  private Command tesCom;

  @Override
  public void robotInit() {
    xkeys = Xkey60.getInstance();
    swerve = Swerve.getInstance();
    escalator = EEscalator.getInstance();
    intake = Intake.getInstance();
    LEDs.getInstance(this);
    LL.portforward();
    LL.BOTTOM.CAMvision();
    LL.BOTTOM.STREAMstandard();
    LL.BOTTOM.setPipeline(0);
    LL.BOTTOM.LEDoff();
    LL.TOP.LEDon();
    LL.TOP.STREAMstandard();
    LL.TOP.CAMvision();
    Config.getInstance();
    OI.getInstance();
    globalTimer = GlobalTimer.getInstance();
    globalState = GlobalState.getInstance();
    swerveFixTimer = new Timer();
    autoSelector = AutoSelector.getInstance();
    tesCom = new EEscalatorBringup();
    // teleop = new EEscalatorBringup();
    teleop = new RobotTeleop();
    swerve.setDefaultCommand(teleop);
    escalator.setDefaultCommand(teleop);
    intake.setDefaultCommand(teleop);

    controlsPeriodic = new MasterControlLoop();

    addPeriodic(controlsPeriodic, Constants.controlDt_s, Constants.controlDt_s / 2.0);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    globalState.disableConeBeam = xkeys.disableConeBeam();
    globalState.disableBallBeam = xkeys.disableCubeBeam();

    SmartDashboard.putBoolean("disableConeBeam", globalState.disableConeBeam);
    SmartDashboard.putBoolean("disableBallBeam", globalState.disableBallBeam);

    if (RobotBase.isSimulation())
      NetworkTableInstance.getDefault().flush();
  }

  @Override
  public void disabledInit() {
    swerve.resetEncoders();
    swerveFixTimer.stop();
    swerveFixTimer.reset();
    swerveFixTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    // TODO rolling avg or smth
    globalState.pollFMS();
    if (swerveFixTimer.get() > 1.0) {
      swerve.resetEncoders();
      swerveFixTimer.reset();
    }
    autoSelector.disabledPeriodic();
    globalState.pollFMS();
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    teleop.cancel();
    Config.getInstance().pigeon.resetHeading(180);
    escalator.zeroEscalator();
    escalator.zeroLinx();
    escalator.zeroWristStowed(); // if not have abs, this saves our ass
    escalator.zeroWristAbs();
    autoSelector.getAutoCommand().schedule();
    globalTimer.start();
  }

  @Override
  public void autonomousPeriodic() {
    // logger.append();
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    var cmd = autoSelector.getAutoCommand();
    if (cmd != null) {
      cmd.cancel();
    }
    if (!globalTimer.isActive()) {
      globalTimer.start();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    // CommandScheduler.getInstance().cancelAll();
    tesCom.schedule();
  }

  @Override
  public void testPeriodic() {
    // logger.append();
  }

  @Override
  public void testExit() {
  }
}
