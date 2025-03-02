// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

import choreo.auto.AutoChooser;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FeatureFlags;
import frc.robot.commands.AutoRoutines;
import frc.robot.sim.SimMechs;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.StructureState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorIOSim;
import frc.robot.subsystems.endeffector.EndEffectorIOTalonFX;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.utils.MappedXboxController;
import frc.robot.utils.autoaim.AlgaeIntakeTargets;
import frc.robot.utils.autoaim.AutoAim;
import frc.robot.utils.autoaim.CoralTargets;
import frc.robot.utils.autoaim.SourceIntakeTargets;
import frc.robot.utils.ratelimiter.AdaptiveSlewRateLimiter;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;

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

  private final Elevator elevator =
      new Elevator(true, Utils.isSimulation() ? new ElevatorIOSim() : new ElevatorIOTalonFX());

  private final Arm arm = new Arm(true, Utils.isSimulation() ? new ArmIOSim() : new ArmIOTalonFX());
  private final EndEffector endEffector =
      new EndEffector(
          true, Utils.isSimulation() ? new EndEffectorIOSim() : new EndEffectorIOTalonFX());

  private final Superstructure superstructure = new Superstructure(elevator, endEffector, arm);
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
    m_autoRoutines = new AutoRoutines(drivetrain.createAutoFactory(drivetrain::trajLogger));
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

    m_operatorController
        .x("Preset for source")
        .onTrue(superstructure.setState(Superstructure.StructureState.PRESOURCE));
    m_operatorController
        .b("Home everything")
        .onTrue(superstructure.setState(Superstructure.StructureState.PREHOME));
    m_operatorController
        .a("Dealgae L2")
        .onTrue(superstructure.setState(Superstructure.StructureState.DEALGAE_L2));
    m_operatorController
        .y("Dealgae L3")
        .onTrue(superstructure.setState(Superstructure.StructureState.DEALGAE_L3));

    m_operatorController
        .povUp("L4 Preset")
        .onTrue(superstructure.setState(Superstructure.StructureState.L4));
    m_operatorController
        .povRight("L3 Preset")
        .onTrue(superstructure.setState(Superstructure.StructureState.L3));
    m_operatorController
        .povDown("L2 Preset")
        .onTrue(superstructure.setState(Superstructure.StructureState.L2));
    m_operatorController
        .povLeft("L1 Preset")
        .onTrue(superstructure.setState(Superstructure.StructureState.L1));

    m_operatorController
        .rightBumper("Manipulator Side Right")
        .onTrue(superstructure.setManipulatorSide(Superstructure.ManipulatorSide.RIGHT));
    m_operatorController
        .leftBumper("Manipulator Side Left")
        .onTrue(superstructure.setManipulatorSide(Superstructure.ManipulatorSide.LEFT));

    m_operatorController
        .rightTrigger("Score Coral")
        .onTrue(superstructure.setState(Superstructure.StructureState.SCORE_CORAL));
    m_operatorController
        .leftTrigger("Score Algae")
        .onTrue(superstructure.setState(Superstructure.StructureState.SCORE_ALGAE));

    new Trigger(() -> -m_operatorController.getLeftY() > .5)
        .onTrue(superstructure.setState(Superstructure.StructureState.BARGE));
    new Trigger(() -> -m_operatorController.getLeftY() < -.5)
        .onTrue(superstructure.setState(Superstructure.StructureState.PROCESSOR));
    new Trigger(() -> -m_operatorController.getRightY() > .5)
        .onTrue(superstructure.setState(Superstructure.StructureState.CLIMB));
  }

  private void configureChoreoAutoChooser() {

    // Add options to the chooser
    autoChooser.addRoutine("ion know", m_autoRoutines::simplePathAuto);
    autoChooser.addCmd("Wheel Radius Change", () -> drivetrain.wheelRadiusCharacterization(1));
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
    // SmartDashboard.updateValues();
    // Put the auto chooser on the dashboard
    // NodeManager nodeManager =
    // new NodeManager(drivetrain, ,
    // drivetrain.createAutoFactory(drivetrain::trajLogger));
    // ArrayList<Node> nodes = new ArrayList<>();
    // nodes.add(new Node(NodeType.PRELOAD, IntakeLocations.Mid, ScoringLocations.H,
    // ScoringTypes.L1));
    // nodes.add(
    // new Node(
    // NodeType.SCORE_AND_INTAKE,
    // IntakeLocations.Source2,
    // ScoringLocations.A,
    // ScoringTypes.L1));
    // nodes.add(new Node(NodeType.WAIT, Seconds.of(5)));
    // nodes.add(
    // new Node(
    // NodeType.SCORE_AND_INTAKE,
    // IntakeLocations.Source2,
    // ScoringLocations.B,
    // ScoringTypes.L1));
    // autoChooser.addRoutine("test", () -> nodeManager.createAuto(nodes));

    SmartDashboard.putData("auto chooser", autoChooser);

    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
  }

  private void configureSwerve() {
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
    azimuth.HeadingController.setPID(6, 0, 0);

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
                      .withRotationalRate(-m_driverController.getTriggerAxes() * MaxAngularRate)));

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
                      .withRotationalRate(-m_driverController.getTriggerAxes() * MaxAngularRate)));
    }
    m_driverController.povUp().whileTrue(drivetrain.wheelRadiusCharacterization(1));

    m_driverController
        .leftBumper() // TODO: remodify this
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
                            -m_driverController.getTriggerAxes()
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
        .povUp()
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
        .povDown()
        .onTrue(
            drivetrain
                .applyRequest(
                    () ->
                        azimuth
                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withTargetDirection(barge))
                .withTimeout(aziTimeout));

    //    new Trigger(
    //            () -> (m_driverController.getRightY() > 0.1 || m_driverController.getRightX() >
    // 0.1))
    //        .onTrue(
    //            drivetrain
    //                .applyRequest(
    //                    () ->
    //                        azimuth
    //                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
    //                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
    //                            .withTargetDirection(
    //                                getStickAngle(m_driverController).plus(new Rotation2d(105))))
    //                .withTimeout(3));

    m_driverController.y("reset heading").onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    // Auto Align Begin
    // preferably a check to make sure we're not in ALGAE state....
    m_driverController
        .povLeft()
        .whileTrue(
            Commands.parallel( // Both run AutoAlign & check tolerance
                Commands.either( // Based on the FF, select REPULSOR or PIDAUTOAIM auto
                    drivetrain.repulsorCommand(
                        () -> {
                          return CoralTargets.getHandedClosestTarget(
                              drivetrain.questNav.getRobotPose().get().toPose2d(), true);
                        }),
                    AutoAim.translateToPose(
                        drivetrain,
                        () -> {
                          return CoralTargets.getHandedClosestTarget(
                              drivetrain.questNav.getRobotPose().get().toPose2d(), true);
                        }),
                    () -> {
                      return FeatureFlags.kAutoAlignPreferRepulsorPF;
                    }),
                Commands.waitUntil(
                        () ->
                            AutoAim.isInToleranceCoral(
                                drivetrain
                                    .questNav
                                    .getRobotPose()
                                    .get()
                                    .toPose2d())) // Additionally, once we're in tolerance,
                    // rumble
                    // controller
                    .andThen(
                        () -> {
                          m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
                        })
                    .andThen(
                        () -> {
                          m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
                        })));

    // Same as prev, except find the NOT righthanded one.
    m_driverController
        .povRight()
        .whileTrue(
            Commands.parallel( // Both run AutoAlign & check tolerance
                Commands.either( // Based on the FF, select REPULSOR or PIDAUTOAIM auto
                    drivetrain.repulsorCommand(
                        () -> {
                          return CoralTargets.getHandedClosestTarget(
                              drivetrain.questNav.getRobotPose().get().toPose2d(), false);
                        }),
                    AutoAim.translateToPose(
                        drivetrain,
                        () -> {
                          return CoralTargets.getHandedClosestTarget(
                              drivetrain.questNav.getRobotPose().get().toPose2d(), false);
                        }),
                    () -> {
                      return FeatureFlags.kAutoAlignPreferRepulsorPF;
                    }),
                Commands.waitUntil(
                        () ->
                            AutoAim.isInToleranceCoral(
                                drivetrain
                                    .questNav
                                    .getRobotPose()
                                    .get()
                                    .toPose2d())) // Additionally, once we're in tolerance,
                    // rumble
                    // controller
                    .andThen(
                        () -> {
                          m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
                        })
                    .andThen(
                        () -> {
                          m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
                        })));
    // Auto Align end
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void superstructureForceIdle() {
    superstructure.setState(StructureState.IDLE);
  }

  public void periodic() {
    Logger.recordOutput(
        "AutoAim/Targets/Coral",
        Stream.of(CoralTargets.values())
            .map((target) -> CoralTargets.getRobotTargetLocation(target.location))
            .toArray(Pose2d[]::new));
    // Log locations of all autoaim targets
    Logger.recordOutput(
        "AutoAim/Targets/Algae",
        Stream.of(AlgaeIntakeTargets.values())
            .map((target) -> AlgaeIntakeTargets.getRobotTargetLocation(target.location))
            .toArray(Pose2d[]::new));

    Logger.recordOutput(
        "AutoAim/Targets/SourceIntakes",
        Stream.of(SourceIntakeTargets.values())
            .map((target) -> SourceIntakeTargets.getRobotTargetLocation(target.location))
            .toArray(Pose2d[]::new));

    Logger.recordOutput(
        "AutoAim/CoralTarget", CoralTargets.getClosestTarget(drivetrain.getState().Pose));
    Logger.recordOutput(
        "AutoAim/LeftHandedCoralTarget",
        CoralTargets.getHandedClosestTarget(
            drivetrain.questNav.getRobotPose().get().toPose2d(), true));
    Logger.recordOutput(
        "AutoAim/RightHandedCoralTarget",
        CoralTargets.getHandedClosestTarget(
            drivetrain.questNav.getRobotPose().get().toPose2d(), false));
    Logger.recordOutput(
        "AutoAim/NameOfLHCoralTarget",
        CoralTargets.getHandedClosestTargetE(
                drivetrain.questNav.getRobotPose().get().toPose2d(), true)
            .name());
    Logger.recordOutput(
        "AutoAim/NameOfRHCoralTarget",
        CoralTargets.getHandedClosestTargetE(
                drivetrain.questNav.getRobotPose().get().toPose2d(), false)
            .name());
    Logger.recordOutput(
        "AutoAim/AlgaeIntakeTarget",
        AlgaeIntakeTargets.getClosestTarget(drivetrain.questNav.getRobotPose().get().toPose2d()));
    superstructure.periodic();
  }
}
