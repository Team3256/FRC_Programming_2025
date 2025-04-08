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
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FeatureFlags;
import frc.robot.commands.AutoRoutines;
import frc.robot.sim.SimMechs;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.ManipulatorSide;
import frc.robot.subsystems.Superstructure.StructureState;
import frc.robot.subsystems.algaearm.AlgaeArm;
import frc.robot.subsystems.algaearm.AlgaeArmTalonFX;
import frc.robot.subsystems.algaerollers.AlgaeRoller;
import frc.robot.subsystems.algaerollers.AlgaeRollerIOTalonFX;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorIOSim;
import frc.robot.subsystems.endeffector.EndEffectorIOTalonFX;
import frc.robot.subsystems.led.IndicatorAnimation;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.utils.MappedXboxController;
import frc.robot.utils.autoaim.AutoAim;
import frc.robot.utils.autoaim.CoralTargets;
import frc.robot.utils.ratelimiter.AdaptiveSlewRateLimiter;

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
  private final AlgaeRoller algaeRoller = new AlgaeRoller(true, new AlgaeRollerIOTalonFX());
  private final EndEffector endEffector =
      new EndEffector(
          true, Utils.isSimulation() ? new EndEffectorIOSim() : new EndEffectorIOTalonFX());

  private final Climb climb = new Climb(true, new ClimbIOTalonFX());
  private final AlgaeArm algaeArm = new AlgaeArm(true, new AlgaeArmTalonFX());
  private final Superstructure superstructure =
      new Superstructure(elevator, endEffector, arm, algaeArm, algaeRoller);
  private final LED leds = new LED();

  private final Vision vision =
      new Vision(
          drivetrain::addPhotonEstimate,
          Utils.isSimulation()
              ? new VisionIOPhotonVisionSim(
                  VisionConstants.leftCam,
                  VisionConstants.robotToLeftCam,
                  () -> drivetrain.getState().Pose)
              : new VisionIOPhotonVision(VisionConstants.leftCam, VisionConstants.robotToLeftCam),
          Utils.isSimulation()
              ? new VisionIOPhotonVisionSim(
                  VisionConstants.rightCam,
                  VisionConstants.robotToRightCam,
                  () -> drivetrain.getState().Pose)
              : new VisionIOPhotonVision(VisionConstants.rightCam, VisionConstants.robotToRightCam),
          Utils.isSimulation()
              ? new VisionIOPhotonVisionSim(
                  VisionConstants.backCam,
                  VisionConstants.robotToBackCam,
                  () -> drivetrain.getState().Pose)
              : new VisionIOPhotonVision(VisionConstants.backCam, VisionConstants.robotToBackCam),
          Utils.isSimulation()
              ? new VisionIOPhotonVisionSim(
                  VisionConstants.frontCam,
                  VisionConstants.robotToFrontCam,
                  () -> drivetrain.getState().Pose)
              : new VisionIOPhotonVision(
                  VisionConstants.frontCam, VisionConstants.robotToFrontCam));
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

  private final Trigger autoAlignTrigger =
      new Trigger(
          () -> {
            return AutoAim.isInToleranceCoral(drivetrain.getState().Pose);
          });

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureOperatorBinds();
    m_autoRoutines =
        new AutoRoutines(
            drivetrain.createAutoFactory(drivetrain::trajLogger),
            elevator,
            arm,
            endEffector,
            drivetrain);
    configureChoreoAutoChooser();
    CommandScheduler.getInstance().registerSubsystem(drivetrain);
    configureSwerve();
    configureLEDs();
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

  // sets up LEDs & rumble
  private void configureLEDs() {
    leds._animate(IndicatorAnimation.Default);
    // leds.setDefaultCommand(leds.animate(IndicatorAnimation.Default).ignoringDisable(true));
    superstructure
        .coralBeamBreak()
        .whileTrue(leds.animate(IndicatorAnimation.CoralIntaken).ignoringDisable(true));
    autoAlignTrigger.whileTrue(
        Commands.run(
                () -> {
                  m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
                })
            .finallyDo(
                () -> {
                  m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
                }));
    autoAlignTrigger.whileTrue(leds.animate(IndicatorAnimation.AutoAligned));
    // autoAlignTrigger.whileTrue(new PrintCommand("AA TRIGGER!!!!").repeatedly());

  }

  private void configureOperatorBinds() {

    m_operatorController
        .x("Preset for source")
        .onTrue(superstructure.setState(StructureState.PRESOURCE));
    m_operatorController
        .b("Home everything")
        .onTrue(superstructure.setState(StructureState.PREHOME));
    m_operatorController.a("Dealgae L2").onTrue(superstructure.setState(StructureState.DEALGAE_L2));
    m_operatorController.y("Dealgae L3").onTrue(superstructure.setState(StructureState.DEALGAE_L3));

    m_operatorController.povUp("L4 Preset").onTrue(superstructure.setState(StructureState.L4));
    m_operatorController.povRight("L3 Preset").onTrue(superstructure.setState(StructureState.L3));
    m_operatorController.povDown("L2 Preset").onTrue(superstructure.setState(StructureState.L2));
    m_operatorController
        .povLeft("Ground Algae")
        .onTrue(superstructure.setState(StructureState.GROUND_ALGAE));
    m_operatorController
        .rightBumper("Manipulator Side Right")
        .onTrue(superstructure.setManipulatorSide(ManipulatorSide.RIGHT));
    m_operatorController
        .leftBumper("Manipulator Side Left")
        .onTrue(superstructure.setManipulatorSide(ManipulatorSide.LEFT));

    m_operatorController
        .rightTrigger("Score Coral")
        .onTrue(superstructure.setState(StructureState.SCORE_CORAL));
    m_operatorController
        .leftTrigger("Score Algae")
        .onTrue(superstructure.setState(StructureState.SCORE_ALGAE));

    new Trigger(() -> -m_operatorController.getLeftY() > .5)
        .onTrue(superstructure.setState(StructureState.BARGE));
    new Trigger(() -> -m_operatorController.getLeftY() < -.5)
        .onTrue(superstructure.setState(StructureState.PROCESSOR));

    new Trigger(() -> m_operatorController.getLeftX() > .5)
        .onTrue(superstructure.setState(StructureState.CLIMB));

    new Trigger(() -> m_operatorController.getRightX() > .5)
        .onTrue(climb.setVoltage(ClimbConstants.kUpVoltage))
        .or(new Trigger(() -> m_operatorController.getRightX() < -.5))
        .onFalse(climb.setVoltage(0));
    new Trigger(() -> m_operatorController.getRightX() < -.5)
        .onTrue(climb.setVoltage(ClimbConstants.kDownVoltage));

    new Trigger(() -> -m_operatorController.getRightY() > .5)
        .whileTrue(new ScheduleCommand(elevator.setPosition(() -> elevator.getPosition() + .1)))
        .toggleOnFalse(elevator.setPosition(elevator::getPosition));
    new Trigger(() -> -m_operatorController.getRightY() < -.5)
        .whileTrue(new ScheduleCommand(elevator.setPosition(() -> elevator.getPosition() - .1)))
        .toggleOnFalse(elevator.setPosition(elevator::getPosition));
  }

  private void configureChoreoAutoChooser() {

    // Add options to the chooser
    autoChooser.addCmd("Wheel Radius Change", () -> drivetrain.wheelRadiusCharacterization(1));
    autoChooser.addRoutine("l4CenterPreload_H", m_autoRoutines::l4PreloadH);
    autoChooser.addRoutine("l4CenterPreload_G", m_autoRoutines::l4PreloadG);
    autoChooser.addRoutine("mobilityTop", m_autoRoutines::mobilityTop);
    autoChooser.addRoutine("mobilityBottom", m_autoRoutines::mobilityBottom);
    autoChooser.addRoutine(
        "l4CenterPreloadRightSource1", m_autoRoutines::l4CenterPreloadRightSource1);
    autoChooser.addRoutine(
        "l4CenterPreloadRightSource2", m_autoRoutines::l4CenterPreloadRightSource2);
    autoChooser.addRoutine(
        "l4RightPreloadRightSource2", m_autoRoutines::l4RightPreloadRightSource2);
    autoChooser.addRoutine("dealgae2LeftPreloadL4_H", m_autoRoutines::dealgae2LeftPreloadL4H);

    SmartDashboard.putData("auto chooser", autoChooser);

    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
  }

  private void configureSwerve() {
    // LinearVelocity is a vector, so we need to get the magnitude
    final double MaxSpeed = TunerConstants.kSpeedAt12Volts.magnitude();
    final double MaxAngularRate = 1.5 * Math.PI;
    final double SlowMaxSpeed = MaxSpeed * 0.3;
    final double SlowMaxAngular = MaxAngularRate * 0.4;

    SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(0.15 * MaxSpeed)
            .withRotationalRate(0.15 * MaxAngularRate);

    SwerveRequest.ApplyRobotSpeeds driveAlt = new SwerveRequest.ApplyRobotSpeeds();

    SwerveRequest.PointWheelsAt lockHoriz = new SwerveRequest.PointWheelsAt();

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
                              -m_driverController.getLeftY() * MaxSpeed)) // Drive
                      // -y
                      // is
                      // forward
                      .withVelocityY(
                          swerveVelYRateLimiter.calculate(
                              -m_driverController.getLeftX() * MaxSpeed))
                      .withRotationalRate(-m_driverController.getTriggerAxes() * MaxAngularRate)));

    } else {
      drivetrain.setDefaultCommand(
          // Drivetrain will execute this command periodically
          drivetrain.applyRequest(
              () ->
                  drive
                      .withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive
                      // forward
                      // with
                      // negative
                      // Y
                      // (forward)
                      .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                      .withRotationalRate(-m_driverController.getTriggerAxes() * MaxAngularRate)));
    }

    // m_driverController.povUp().whileTrue(drivetrain.wheelRadiusCharacterization(1));

    m_driverController
        .leftBumper()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(-m_driverController.getLeftY() * SlowMaxSpeed) // Drive
                        // forward
                        // with
                        // negative
                        // Y
                        // (forward)
                        .withVelocityY(-m_driverController.getLeftX() * SlowMaxSpeed) // Drive
                        // left
                        // with
                        // negative
                        // X
                        // (left)
                        .withRotationalRate(
                            -m_driverController.getTriggerAxes() * SlowMaxAngular) // Drive
                // counterclockwise
                // with
                // negative
                // X
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
                .withTimeout(aziTimeout2));

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
                .withTimeout(aziTimeout2));

    //
    //    m_driverController
    //        .rightBumper()
    //        .onTrue(
    //            drivetrain
    //                .applyRequest(
    //                    () ->
    //                        azimuth
    //                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
    //                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
    //                            .withTargetDirection(barge))
    //                .withTimeout(aziTimeout2));

    m_driverController
        .povUp()
        .onTrue(
            drivetrain
                .applyRequest(
                    () ->
                        azimuth
                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withTargetDirection(processorClose)) // doubles as climb facing cage
                .withTimeout(aziTimeout2));

    m_driverController
        .povDown()
        .onTrue(
            drivetrain
                .applyRequest(
                    () ->
                        azimuth
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withTargetDirection(
                                processorFar)) // double as climb from opposite side facing DS
                .withTimeout(aziTimeout2));

    m_driverController
        .a()
        .whileTrue(
            drivetrain.pidXLocked(
                () -> {
                  return 7.75;
                },
                () -> {
                  return -m_driverController.getLeftX() * MaxSpeed;
                },
                () -> {
                  return -m_driverController.getTriggerAxes() * MaxAngularRate;
                }));

    new Trigger(() -> (m_driverController.getRightY() > 0.3))
        .onTrue(
            drivetrain
                .applyRequest(
                    () ->
                        azimuth
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withTargetDirection(bargeClose))
                .withTimeout(aziTimeout2));

    new Trigger(() -> (m_driverController.getRightY() < -0.3))
        .onTrue(
            drivetrain
                .applyRequest(
                    () ->
                        azimuth
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withTargetDirection(bargeFar))
                .withTimeout(aziTimeout2));

    m_driverController.y("reset heading").onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    //
    // new Trigger(
    // () ->
    // ((m_driverController.getRightY() > 0.1 || m_driverController.getRightX() >
    // 0.1)
    // && (Math.atan2(m_driverController.getRightY(),
    // m_driverController.getRightX())
    // < 1.523 + 0.0872665) // upper
    // // bound
    // && ((Math.atan2(m_driverController.getRightY(),
    // m_driverController.getRightX())
    // > 1.523 - 0.0872665)))) // lower
    // // bound
    // .onTrue(
    // drivetrain
    // .applyRequest(
    // () ->
    // azimuth
    // .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
    // .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
    // .withTargetDirection(reefAB))
    // .withTimeout(aziTimeout));
    //
    // new Trigger(
    // () ->
    // ((m_driverController.getRightY() > 0.1 || m_driverController.getRightX() >
    // 0.1)
    // && (Math.atan2(m_driverController.getRightY(),
    // m_driverController.getRightX())
    // < 0.483 + 0.0872665)
    // && ((Math.atan2(m_driverController.getRightY(),
    // m_driverController.getRightX())
    // > 0.483 - 0.0872665))))
    // .onTrue(
    // drivetrain
    // .applyRequest(
    // () ->
    // azimuth
    // .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
    // .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
    // .withTargetDirection(reefCD))
    // .withTimeout(aziTimeout));
    //
    // new Trigger(
    // () ->
    // ((m_driverController.getRightY() > 0.1 || m_driverController.getRightX() >
    // 0.1)
    // && (Math.atan2(m_driverController.getRightY(),
    // m_driverController.getRightX())
    // < -0.612 + 0.0872665)
    // && ((Math.atan2(m_driverController.getRightY(),
    // m_driverController.getRightX())
    // > -0.612 - 0.0872665))))
    // .onTrue(
    // drivetrain
    // .applyRequest(
    // () ->
    // azimuth
    // .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
    // .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
    // .withTargetDirection(reefEF))
    // .withTimeout(aziTimeout));
    //
    // new Trigger(
    // () ->
    // ((m_driverController.getRightY() > 0.1 || m_driverController.getRightX() >
    // 0.1)
    // && (Math.atan2(m_driverController.getRightY(),
    // m_driverController.getRightX())
    // < -1.534 + 0.0872665)
    // && ((Math.atan2(m_driverController.getRightY(),
    // m_driverController.getRightX())
    // > -1.534 - 0.0872665))))
    // .onTrue(
    // drivetrain
    // .applyRequest(
    // () ->
    // azimuth
    // .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
    // .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
    // .withTargetDirection(reefGH))
    // .withTimeout(aziTimeout));
    //
    // new Trigger(
    // () ->
    // ((m_driverController.getRightY() > 0.1 || m_driverController.getRightX() >
    // 0.1)
    // && (Math.atan2(m_driverController.getRightY(),
    // m_driverController.getRightX())
    // < -2.437 + 0.0872665)
    // && ((Math.atan2(m_driverController.getRightY(),
    // m_driverController.getRightX())
    // > -2.437 - 0.0872665))))
    // .onTrue(
    // drivetrain
    // .applyRequest(
    // () ->
    // azimuth
    // .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
    // .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
    // .withTargetDirection(reefIJ))
    // .withTimeout(aziTimeout));
    //
    // new Trigger(
    // () ->
    // ((m_driverController.getRightY() > 0.1 || m_driverController.getRightX() >
    // 0.1)
    // && (Math.atan2(m_driverController.getRightY(),
    // m_driverController.getRightX())
    // < 2.793 + 0.0872665)
    // && ((Math.atan2(m_driverController.getRightY(),
    // m_driverController.getRightX())
    // > 2.793 - 0.0872665))))
    // .onTrue(
    // drivetrain
    // .applyRequest(
    // () ->
    // azimuth
    // .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
    // .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
    // .withTargetDirection(reefKL))
    // .withTimeout(aziTimeout));
    //
    // Auto Align Begin
    // preferably a check to make sure we're not in ALGAE state....

    m_driverController
        .povLeft()
        .whileTrue(
            Commands.parallel(
                drivetrain.pidToPose(
                    () -> CoralTargets.getHandedClosestTarget(drivetrain.getState().Pose, true)),
                leds.animate(IndicatorAnimation.AutoAlignRunning)));

    // Same as prev, except find the NOT lefthanded one.
    m_driverController
        .povRight()
        .whileTrue( // Both run AutoAlign & check tolerance
            Commands.parallel(
                drivetrain.pidToPose(
                    () -> CoralTargets.getHandedClosestTarget(drivetrain.getState().Pose, false)),
                leds.animate(IndicatorAnimation.AutoAlignRunning)));
    // Auto Align end
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void periodic() {
    // Logger.recordOutput(
    // "Stick Angle Radians",
    // Math.atan2(m_driverController.getRightY(), m_driverController.getRightX()));
    // Logger.recordOutput(
    // "AutoAim/Targets/Coral",
    // Stream.of(CoralTargets.values())
    // .map((target) -> CoralTargets.getRobotTargetLocation(target.location))
    // .toArray(Pose2d[]::new));
    // // Log locations of all autoaim targets
    // Logger.recordOutput(
    // "AutoAim/Targets/Algae",
    // Stream.of(AlgaeIntakeTargets.values())
    // .map((target) -> AlgaeIntakeTargets.getRobotTargetLocation(target.location))
    // .toArray(Pose2d[]::new));
    //
    // Logger.recordOutput(
    // "AutoAim/Targets/SourceIntakes",
    // Stream.of(SourceIntakeTargets.values())
    // .map((target) -> SourceIntakeTargets.getRobotTargetLocation(target.location))
    // .toArray(Pose2d[]::new));
    //
    // Logger.recordOutput(
    // "AutoAim/CoralTarget",
    // CoralTargets.getClosestTarget(drivetrain.getState().Pose));
    // Logger.recordOutput(
    // "AutoAim/LeftHandedCoralTarget",
    // CoralTargets.getHandedClosestTarget(drivetrain.getState().Pose, true));
    // Logger.recordOutput(
    // "AutoAim/RightHandedCoralTarget",
    // CoralTargets.getHandedClosestTarget(drivetrain.getState().Pose, false));
    // Logger.recordOutput(
    // "AutoAim/NameOfLHCoralTarget",
    // CoralTargets.getHandedClosestTargetE(drivetrain.getState().Pose,
    // true).name());
    // Logger.recordOutput(
    // "AutoAim/NameOfRHCoralTarget",
    // CoralTargets.getHandedClosestTargetE(drivetrain.getState().Pose,
    // false).name());
    // Logger.recordOutput(
    // "AutoAim/AlgaeIntakeTarget",
    // AlgaeIntakeTargets.getClosestTarget(drivetrain.getState().Pose));
    superstructure.periodic();
  }
}
