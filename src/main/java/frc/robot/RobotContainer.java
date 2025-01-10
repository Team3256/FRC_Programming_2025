// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;

import java.lang.ModuleLayer.Controller;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FeatureFlags;
import frc.robot.commands.AutoRoutines;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.utils.MappedXboxController;
import frc.robot.utils.ratelimiter.AdaptiveSlewRateLimiter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final MappedXboxController m_driverController = new MappedXboxController(
      ControllerConstants.kDriverControllerPort, "driver");
  public final MappedXboxController m_operatorController = new MappedXboxController(
      ControllerConstants.kOperatorControllerPort, "operator");

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /* Swerve Rate Limiting */
  private final AdaptiveSlewRateLimiter swerveVelXRateLimiter = new AdaptiveSlewRateLimiter(
      ControllerConstants.DriverConstants.kSwerveVelXAccelRateLimit,
      ControllerConstants.DriverConstants.kSwerveVelXDecelRateLimit);
  private final AdaptiveSlewRateLimiter swerveVelYRateLimiter = new AdaptiveSlewRateLimiter(
      ControllerConstants.DriverConstants.kSwerveVelYAccelRateLimit,
      ControllerConstants.DriverConstants.kSwerveVelYDecelRateLimit);
  private final AdaptiveSlewRateLimiter swerveAngVelRateLimiter = new AdaptiveSlewRateLimiter(
      ControllerConstants.DriverConstants.kSwerveAngVelAccelRateLimit,
      ControllerConstants.DriverConstants.kSwerveAngVelDecelRateLimit);

  private final AutoRoutines m_autoRoutines;
  private final AutoChooser autoChooser = new AutoChooser();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_autoRoutines = new AutoRoutines(drivetrain.createAutoFactory(drivetrain::trajLogger));
    configureChoreoAutoChooser();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b("Example
    // method").whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  private void configureChoreoAutoChooser() {

    // Add options to the chooser
    autoChooser.addRoutine("Example Routine", m_autoRoutines::simplePathAuto);

    // Put the auto chooser on the dashboard
    SmartDashboard.putData(autoChooser);

    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
  }

  private void configureSwerve() {
    // LinearVelocity is a vector, so we need to get the magnitude
    double MaxSpeed = TunerConstants.kSpeedAt12Volts.magnitude();
    double MaxAngularRate = 1.5 * Math.PI; // My drivetrain
    double SlowMaxSpeed = MaxSpeed * 0.3;
    double SlowMaxAngular = MaxAngularRate * 0.3;

    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
    if (FeatureFlags.kSwerveAccelerationLimitingEnabled) {
      drivetrain.applyRequest(
          () -> drive
              .withVelocityX(
                  swerveVelXRateLimiter.calculate(
                      m_driverController.getLeftY() * MaxSpeed)) // Drive -y is
              // forward
              .withVelocityY(
                  swerveVelYRateLimiter.calculate(
                      m_driverController.getLeftX() * MaxSpeed)) // Drive -x is
              // left
              .withRotationalRate(
                  swerveAngVelRateLimiter.calculate(
                      -m_driverController.getRightX() * MaxAngularRate)));
    } else {
      drivetrain.applyRequest(
          () -> drive
              .withVelocityX(m_driverController.getLeftY() * MaxSpeed) // Drive -y is forward
              .withVelocityY(m_driverController.getLeftX() * MaxSpeed) // Drive -x is left
              .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate));
    }
  }
}
