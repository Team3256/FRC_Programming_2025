// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.swerve.AngleCalculator.getStickAngle;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

import choreo.auto.AutoChooser;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FeatureFlags;
import frc.robot.autogen.*;
import frc.robot.commands.AutoRoutines;
import frc.robot.sim.SimMechs;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.rollers.Roller;
import frc.robot.subsystems.rollers.RollerIOTalonFX;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.ResetPose;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.utils.MappedXboxController;
import frc.robot.utils.ratelimiter.AdaptiveSlewRateLimiter;
import java.util.ArrayList;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final MappedXboxController m_driverController =
      new MappedXboxController(ControllerConstants.kDriverControllerPort, "driver");
  public final MappedXboxController m_operatorController =
      new MappedXboxController(ControllerConstants.kOperatorControllerPort, "operator");

  private final Telemetry logger =
      new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private Rotation2d finalAutoHeading;

  private final Roller roller = new Roller(true, new RollerIOTalonFX());

  private final Arm arm = new Arm(true, Utils.isSimulation() ? new ArmIOSim() : new ArmIOTalonFX());
  private final Climb climb = new Climb(true, new ClimbIOTalonFX());

  /* Swerve Rate Limiting */
  private final AdaptiveSlewRateLimiter swerveVelXRateLimiter =
      new AdaptiveSlewRateLimiter(
          ControllerConstants.DriverConstants.kSwerveVelXAccelRateLimit,
          ControllerConstants.DriverConstants.kSwerveVelXDecelRateLimit);
  private final AdaptiveSlewRateLimiter swerveVelYRateLimiter =
      new AdaptiveSlewRateLimiter(
          ControllerConstants.DriverConstants.kSwerveVelYAccelRateLimit,
          ControllerConstants.DriverConstants.kSwerveVelYDecelRateLimit);
  private final AdaptiveSlewRateLimiter swerveAngVelRateLimiter =
      new AdaptiveSlewRateLimiter(
          ControllerConstants.DriverConstants.kSwerveAngVelAccelRateLimit,
          ControllerConstants.DriverConstants.kSwerveAngVelDecelRateLimit);

  private final AutoRoutines m_autoRoutines;
  private AutoChooser autoChooser = new AutoChooser();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_autoRoutines = new AutoRoutines(drivetrain.createAutoFactory(drivetrain::trajLogger), roller);
    configureChoreoAutoChooser();
    CommandScheduler.getInstance().registerSubsystem(drivetrain);
    configureSwerve();
    if (Utils.isSimulation()) {
      SimMechs.getInstance().publishToNT();
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b("Example
    // method").whileTrue(m_exampleSubsystem.exampleMethodCommand());

    m_operatorController
        .rightBumper("s")
        .onTrue(Commands.runOnce(() -> drivetrain.resetPose(new Pose2d())));
    // m_operatorController.a("ds").onTrue(roller.setRollerVoltage(6));
    // m_operatorController.b("dsa").onTrue(roller.setRollerVoltage(-6));
    // m_operatorController.y("off").onTrue(roller.off());
    // m_operatorController
    //     .rightBumper("s")
    //     .onTrue(Commands.runOnce(() -> drivetrain.resetPose(new Pose2d())));
  }

  private void configureChoreoAutoChooser() {

    // Add options to the chooser
    autoChooser.addRoutine("ion know", m_autoRoutines::simplePathAuto);
    autoChooser.addCmd(
        "Wheel Radius Change",
        () ->
            drivetrain.wheelRadiusCharacterization(
                SwerveConstants.wheelRadiusMaxVelocity, SwerveConstants.wheelRadiusMaxRampRate));
    autoChooser.addCmd(
        "SysID forward translation dynamic",
        () -> drivetrain.sysIdTranslationDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "SysID backward translation dynamic",
        () -> drivetrain.sysIdTranslationDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addCmd(
        "SysID forward translation quasitastic",
        () -> drivetrain.sysIdTranslationQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "SysID forward translation quasitastic",
        () -> drivetrain.sysIdTranslationQuasistatic(SysIdRoutine.Direction.kReverse));

    autoChooser.addCmd(
        "SysID forward rotation dynamic",
        () -> drivetrain.sysIdRotationDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "SysID backward rotation dynamic",
        () -> drivetrain.sysIdRotationDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addCmd(
        "SysID forward rotation quasitastic",
        () -> drivetrain.sysIdTranslationQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "SysID forward rotation quasitastic",
        () -> drivetrain.sysIdRotationQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addCmd("Start Signal Logger", () -> Commands.runOnce(SignalLogger::start));
    autoChooser.addCmd("End Signal Logger", () -> Commands.runOnce(SignalLogger::stop));
    //    SmartDashboard.updateValues();
    // Put the auto chooser on the dashboard
    NodeManager nodeManager =
        new NodeManager(drivetrain, roller, drivetrain.createAutoFactory(drivetrain::trajLogger));
    ArrayList<Node> nodes = new ArrayList<>();
    nodes.add(new Node(NodeType.PRELOAD, IntakeLocations.Mid, ScoringLocations.H, ScoringTypes.L1));
    nodes.add(
        new Node(
            NodeType.SCORE_AND_INTAKE,
            IntakeLocations.Source2,
            ScoringLocations.A,
            ScoringTypes.L1));
    nodes.add(new Node(NodeType.WAIT, Seconds.of(5)));
    nodes.add(
        new Node(
            NodeType.SCORE_AND_INTAKE,
            IntakeLocations.Source2,
            ScoringLocations.B,
            ScoringTypes.L1));
    autoChooser.addRoutine("test", () -> nodeManager.createAuto(nodes));

    SmartDashboard.putData("auto chooser", autoChooser);

    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
  }

  public void updateAngles() {
    this.finalAutoHeading = this.drivetrain.getCurrentHeading();
  }

  public void configureSwerve() {
    // LinearVelocity is a vector, so we need to get the magnitude
    final double MaxSpeed = TunerConstants.kSpeedAt12Volts.magnitude();
    final double MaxAngularRate = 1.5 * Math.PI;
    final double SlowMaxSpeed = MaxSpeed * 0.3;
    final double SlowMaxAngular = MaxAngularRate * 0.3;

    SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(0.15 * MaxSpeed)
            .withRotationalRate(0.15 * MaxAngularRate);

    SwerveRequest.ApplyRobotSpeeds driveAlt = new SwerveRequest.ApplyRobotSpeeds();

    SwerveRequest.FieldCentricFacingAngle azimuth =
        new SwerveRequest.FieldCentricFacingAngle().withDeadband(0.15 * MaxSpeed);

    azimuth.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    azimuth.HeadingController.setPID(8, 250, 2);

    if (FeatureFlags.kSwerveAccelerationLimitingEnabled) {
      drivetrain.setDefaultCommand(
          drivetrain.applyRequest(
              () ->
                  drive
                      .withVelocityX(
                          swerveVelXRateLimiter.calculate(
                              m_driverController.getLeftY() * MaxSpeed)) // Drive -y is forward
                      .withVelocityY(
                          swerveVelYRateLimiter.calculate(m_driverController.getLeftX() * MaxSpeed))
                      .withRotationalRate(m_driverController.getTriggerAxes() * MaxAngularRate)));

    } else {
      drivetrain.setDefaultCommand(
          // Drivetrain will execute this command periodically
          drivetrain.applyRequest(
              () ->
                  drive
                      .withVelocityX(
                          -m_driverController.getLeftY()
                              * MaxSpeed) // Drive forward with negative Y (forward)
                      .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                      .withRotationalRate(m_driverController.getTriggerAxes() * MaxAngularRate)));
    }

    m_driverController
        .leftBumper()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(
                            -m_driverController.getLeftY()
                                * SlowMaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(
                            -m_driverController.getLeftX()
                                * SlowMaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(
                            m_driverController.getTriggerAxes()
                                * SlowMaxAngular) // Drive counterclockwise with negative X
                // (left)
                ));

    m_driverController
        .x()
        .onTrue(
            drivetrain
                .applyRequest(
                    () ->
                        azimuth
                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withTargetDirection(sourceLeft1))
                .withTimeout(aziTimeout));

    m_driverController
        .b()
        .onTrue(
            drivetrain
                .applyRequest(
                    () ->
                        azimuth
                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withTargetDirection(sourceRight2))
                .withTimeout(aziTimeout));

    m_driverController
        .a()
        .onTrue(
            drivetrain
                .applyRequest(
                    () ->
                        azimuth
                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withTargetDirection(hang))
                .withTimeout(aziTimeout));

    m_driverController
        .povUp()
        .onTrue(
            drivetrain
                .applyRequest(
                    () ->
                        azimuth
                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withTargetDirection(finalAutoHeading))
                .withTimeout(aziTimeout));

    new Trigger(
            () -> (m_driverController.getRightY() > 0.15 || m_driverController.getRightX() > 0.15))
        .onTrue(
            drivetrain
                .applyRequest(
                    () ->
                        azimuth
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                            .withTargetDirection(getStickAngle(m_driverController)))
                .withTimeout(3));

    m_driverController.y("reset heading").onTrue(new ResetPose(drivetrain));

    drivetrain.registerTelemetry(logger::telemeterize);
  }
}
