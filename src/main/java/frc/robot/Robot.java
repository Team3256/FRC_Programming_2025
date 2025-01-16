// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.NT4PublisherNoFMS;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
@Logged
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    super();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    if (Constants.FeatureFlags.kAdvKitEnabled) {
      configureAdvantageKit();
    }
    if (Constants.FeatureFlags.kEpilogueEnabled) {
      configureEpilogue();
    }
  }

  private void configureEpilogue() {
    // The default is NetworkTables logging,
    // which is what we want
    Epilogue.configure(
        config -> {
          if (isSimulation()) {
            // If running in simulation, then we'd want to re-throw any errors that
            // occur so we can debug and fix them!
            config.errorHandler = ErrorHandler.crashOnError();
          }

          // Change the root data path
          config.root = "Epilogue";

          config.minimumImportance = Constants.Logging.kEpilogueImportance;
        });
    // Epilogue.bind(this);
  }

  private void configureAdvantageKit() {
    if (isReal()) {
      if (Constants.Logging.kLogToUSB) {
        // Note: By default, the WPILOGWriter class writes to a USB stick (at the path
        // of /U/logs) when running on the roboRIO. A FAT32 (sadly not exFAT, which is
        // the generally better format) formatted USB stick must be connected to one of
        // the roboRIO USB ports.
        Logger.addDataReceiver(new WPILOGWriter("/U/wpilogs"));
      } else {
        Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
      }
      Logger.addDataReceiver(new NT4PublisherNoFMS()); // Publish data to NetworkTables
      // Enables power distribution logging
      new PowerDistribution(
          1, ModuleType.kRev); // Ignore this "resource leak"; it was the example code from docs
    } else {
      if (Constants.Logging.kAdvkitUseReplayLogs) {
        setUseTiming(false); // Run as fast as possible
        String logPath =
            LogFileUtil
                .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        // Save outputs to a new log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      } else {
        Logger.addDataReceiver(new NT4Publisher());
      }
    }

    // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    // Disabling deterministic timestamps disallows replay

    // Logger.disableDeterministicTimestamps();

    // Disabling deterministic timestamps (uncommenting the previous line of code)
    // should only be used when all of the following are true:
    // (quoting from
    // https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/DATA-FLOW.md#solution-3)
    // 1. The control logic depends on the exact timestamp within a single loop
    // cycle, like a high precision control loop that is significantly affected by
    // the precise time that it is executed within each (usually 20ms) loop cycle.
    // 2. The sensor values used in the loop cannot be associated with timestamps in
    // an IO implementation. See solution #1.
    // 3. The IO (sensors, actuators, etc) involved in the loop are sufficiently
    // low-latency that the exact timestamp on the RIO is significant. For example,
    // CAN motor controllers are limited by the rate of their CAN frames, so the
    // extra precision on the RIO is insignificant in most cases.

    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME); // Set a metadata value
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.
    Logger.start();

    // The reason why we log build time and other project metadata
    // is so we can easily identify the version of the currently
    // deployed code on the robot
    // It's also recommended ala
    // https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/INSTALLATION.md#gversion-plugin-git-metadata
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
