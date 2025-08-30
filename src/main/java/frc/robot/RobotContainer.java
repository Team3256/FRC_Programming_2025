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
import choreo.util.ChoreoAllianceFlipUtil;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.InternalButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
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
import java.util.List;

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

  private final AutoRoutines m_autoRoutines;
  private AutoChooser autoChooser = new AutoChooser();

  private final Trigger autoAlignedTrigger =
      new Trigger(() -> AutoAim.isInToleranceCoral(drivetrain.getState().Pose));

  private final InternalButton autoAlignRunning = new InternalButton();

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
            algaeArm,
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
  public Pose2d getPose() {
    return drivetrain.getState().Pose;
  }

  public LED getLed() {
    return leds;
  }

  // sets up LEDs & rumble
  private void configureLEDs() {
    leds.setDefaultCommand(leds.animate(IndicatorAnimation.Default));
    superstructure
        .coralBeamBreak()
        .and(autoAlignRunning.negate())
        .and(autoAlignedTrigger.negate())
        .whileTrue(leds.animate(IndicatorAnimation.CoralIntaken));
    superstructure
        .algaeBeamBreak()
        .and(superstructure.coralBeamBreak().negate())
        .and(autoAlignedTrigger.negate())
        .and(autoAlignRunning.negate())
        .whileTrue(leds.animate(IndicatorAnimation.AlgaeIntaken));
    superstructure.climbState().whileTrue(leds.animate(IndicatorAnimation.Climb));
    autoAlignedTrigger.whileTrue(
        Commands.run(
                () -> {
                  m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
                })
            .finallyDo(
                () -> {
                  m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
                }));
    autoAlignedTrigger.whileTrue(leds.animate(IndicatorAnimation.AutoAligned));
    autoAlignRunning
        .and(autoAlignedTrigger.negate())
        .onTrue(leds.animate(IndicatorAnimation.AutoAlignRunning));
    // autoAlignTrigger.whileTrue(new PrintCommand("AA TRIGGER!!!!").repeatedly());

  }

  public Pose2d getClosestAlignment() {
    return drivetrain
        .getState()
        .Pose
        .nearest(
            List.of(
                new Pose2d(7.197261810302734, 2.482149600982666, Rotation2d.fromDegrees(270)),
                ChoreoAllianceFlipUtil.flip(
                    new Pose2d(7.197261810302734, 2.482149600982666, Rotation2d.fromDegrees(270))),
                new Pose2d(7.228638648986816, 4.02569580078125, Rotation2d.fromDegrees(270)),
                ChoreoAllianceFlipUtil.flip(
                    new Pose2d(7.228638648986816, 4.02569580078125, Rotation2d.fromDegrees(270))),
                new Pose2d(7.197261810302734, 5.6082072257995605, Rotation2d.fromDegrees(270)),
                ChoreoAllianceFlipUtil.flip(
                    new Pose2d(
                        7.197261810302734, 5.6082072257995605, Rotation2d.fromDegrees(270)))));
  }

  private void configureOperatorBinds() {

    // source
    m_operatorController
        .x("Preset for source")
        .onTrue(superstructure.setState(StructureState.PRESOURCE));

    // stow everything
    m_operatorController
        .b("Home everything")
        .onTrue(superstructure.setState(StructureState.PREHOME));

    // dealgae states
    m_operatorController.a("Dealgae L2").onTrue(superstructure.setState(StructureState.DEALGAE_L2));
    m_operatorController.y("Dealgae L3").onTrue(superstructure.setState(StructureState.DEALGAE_L3));

    // reef states
    m_operatorController.povUp("L4 Preset").onTrue(superstructure.setState(StructureState.L4));
    m_operatorController.povRight("L3 Preset").onTrue(superstructure.setState(StructureState.L3));
    m_operatorController.povDown("L2 Preset").onTrue(superstructure.setState(StructureState.L2));
    m_operatorController.povLeft("L1 Preset").onTrue(superstructure.setState(StructureState.L1));

    // change end effector side
    m_operatorController
        .rightBumper("Manipulator Side Right")
        .onTrue(superstructure.setManipulatorSide(ManipulatorSide.RIGHT));
    m_operatorController
        .leftBumper("Manipulator Side Left")
        .onTrue(superstructure.setManipulatorSide(ManipulatorSide.LEFT));

    // outtake coral
    m_operatorController
        .rightTrigger("Score Coral")
        .onTrue(superstructure.setState(StructureState.SCORE_CORAL));

    // outtake algae
    m_operatorController
        .leftTrigger("Score Algae")
        .onTrue(superstructure.setState(StructureState.SCORE_ALGAE));

    // set state BARGE
    new Trigger(() -> -m_operatorController.getLeftY() > .5)
        .onTrue(superstructure.setState(StructureState.BARGE));

    // set state PROCESSOR
    new Trigger(() -> -m_operatorController.getLeftY() < -.5)
        .onTrue(superstructure.setState(StructureState.PROCESSOR));

    // set state CLIMB
    new Trigger(() -> m_operatorController.getLeftX() > .5)
        .onTrue(superstructure.setState(StructureState.CLIMB));

    // set state GROUND ALGAE
    new Trigger(() -> m_operatorController.getLeftX() < -.5)
        .onTrue(superstructure.setState(StructureState.GROUND_ALGAE));

    // climb out
    new Trigger(() -> m_operatorController.getRightX() > .5)
        .onTrue(climb.setVoltage(ClimbConstants.kUpVoltage))
        .or(new Trigger(() -> m_operatorController.getRightX() < -.5))
        .onFalse(climb.setVoltage(0));

    // pull climb in
    new Trigger(() -> m_operatorController.getRightX() < -.5)
        .onTrue(climb.setVoltage(ClimbConstants.kDownVoltage));

    // elevator manual up
    new Trigger(() -> -m_operatorController.getRightY() > .5)
        .whileTrue(new ScheduleCommand(elevator.setPosition(() -> elevator.getPosition() + .1)))
        .toggleOnFalse(elevator.setPosition(elevator::getPosition));

    // elevator manual down
    new Trigger(() -> -m_operatorController.getRightY() < -.5)
        .whileTrue(new ScheduleCommand(elevator.setPosition(() -> elevator.getPosition() - .1)))
        .toggleOnFalse(elevator.setPosition(elevator::getPosition));
  }

  private void configureChoreoAutoChooser() {

    // Add options to the chooser
    autoChooser.addCmd("Wheel Radius Change", () -> drivetrain.wheelRadiusCharacterization(1));
    autoChooser.addRoutine("Center 1L4-H Preload", m_autoRoutines::l4PreloadH);
    autoChooser.addRoutine("Center 1L4-G Preload", m_autoRoutines::l4PreloadG);
    autoChooser.addRoutine("Mobility Left", m_autoRoutines::mobilityLeft);
    autoChooser.addRoutine("Mobility Right", m_autoRoutines::mobilityRight);
    autoChooser.addRoutine("Center 3L4-GCD", m_autoRoutines::l4CenterPreloadRightSourceRight);
    autoChooser.addRoutine("Right 3L4-FCD", m_autoRoutines::l4RightPreloadRightSourceRight);
    autoChooser.addRoutine("Left 3L4-IKL", m_autoRoutines::l4LeftPreloadLeftSourceLeft);
    autoChooser.addRoutine("Center 1L4-H 2Barge-GH_IJ", m_autoRoutines::dealgae2LeftPreloadL4H);

    SmartDashboard.putData("auto chooser", autoChooser);

    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
  }

  private void configureSwerve() {

    // Request to drive normally using input for both translation and rotation
    SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(0.15 * MaxSpeed)
            .withRotationalRate(0.15 * MaxAngularRate);

    // Request to control translation, with rotation being controlled by a heading controller
    SwerveRequest.FieldCentricFacingAngle azimuth =
        new SwerveRequest.FieldCentricFacingAngle().withDeadband(0.15 * MaxSpeed);

    // Heading controller to control azimuth rotations
    azimuth.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    azimuth.HeadingController.setPID(6, 0, 0);

    // Default Swerve Command, run periodically every 20ms
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                    .withRotationalRate(-m_driverController.getTriggerAxes() * MaxAngularRate)));

    m_driverController
        .leftBumper("Brake / Slow Mode")
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(-m_driverController.getLeftY() * SlowMaxSpeed)
                        .withVelocityY(-m_driverController.getLeftX() * SlowMaxSpeed)
                        .withRotationalRate(
                            -m_driverController.getTriggerAxes() * SlowMaxAngular)));

    m_driverController
        .x("Azimuth Left Source")
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
        .b("Azimuth Right Source")
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
        .povUp("Processor Close, Climb Facing cage")
        .onTrue(
            drivetrain
                .applyRequest(
                    () ->
                        azimuth
                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withTargetDirection(processorClose)) // doubles as climb facing cage
                .withTimeout(aziTimeout));

    m_driverController
        .povDown("Processor Far, Climb Facing DS")
        .onTrue(
            drivetrain
                .applyRequest(
                    () ->
                        azimuth
                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withTargetDirection(
                                processorFar)) // doubles as climb from opposite side facing DS
                .withTimeout(aziTimeout));

    // Azimuth Barge Close
    new Trigger(() -> (m_driverController.getRightY() < -0.3))
        .onTrue(
            drivetrain
                .applyRequest(
                    () ->
                        azimuth
                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withTargetDirection(bargeClose))
                .withTimeout(aziTimeout));

    // Azimuth Barge Far
    new Trigger(() -> (m_driverController.getRightY() > 0.3))
        .onTrue(
            drivetrain
                .applyRequest(
                    () ->
                        azimuth
                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withTargetDirection(bargeFar))
                .withTimeout(aziTimeout));

    // barge auto align - don't use
    m_driverController
        .rightBumper("Auto Align Barge Close")
        .whileTrue(
            drivetrain.pidXLocked(
                () -> bargeCloseX,
                () -> -m_driverController.getLeftX() * MaxSpeed,
                () -> -m_driverController.getTriggerAxes() * MaxAngularRate));

    m_driverController
        .a("Auto Align Barge Far")
        .whileTrue(
            drivetrain.pidXLocked(
                () -> bargeFarX,
                () -> -m_driverController.getLeftX() * MaxSpeed,
                () -> -m_driverController.getTriggerAxes() * MaxAngularRate));

    // sets the heading to wherever the robot is facing
    // do this with the elevator side of the robot facing YOU
    m_driverController.y("Zero Heading").onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // Auto Align Reef, Left Handed Target (Absolute)
    new Trigger(
            () ->
                ((superstructure.getState() != StructureState.GROUND_ALGAE)
                        && (superstructure.getState() != StructureState.PROCESSOR)
                        && (superstructure.getState()) != StructureState.BARGE)
                    && (m_driverController.povLeft().getAsBoolean()))
        .whileTrue(
            Commands.parallel(
                drivetrain.pidToPose(
                    () -> CoralTargets.getHandedClosestTarget(drivetrain.getState().Pose, true))));

    // Auto Align Reef, Right Handed Target (Absolute)
    new Trigger(
            () ->
                ((superstructure.getState() != StructureState.GROUND_ALGAE)
                        && (superstructure.getState() != StructureState.PROCESSOR)
                        && (superstructure.getState()) != StructureState.BARGE)
                    && (m_driverController.povRight().getAsBoolean()))
        .whileTrue(
            Commands.parallel(
                drivetrain.pidToPose(
                    () -> CoralTargets.getHandedClosestTarget(drivetrain.getState().Pose, false))));

    // Run LEDs simultaneously with Auto Align
    m_driverController
        .povLeft()
        .negate()
        .and(m_driverController.povRight().negate())
        .and(m_driverController.a().negate())
        .and(m_driverController.rightBumper().negate())
        .onTrue(new InstantCommand(() -> autoAlignRunning.setPressed(false)));
    m_driverController
        .povRight()
        .or(m_driverController.povLeft())
        .or(m_driverController.a())
        .or(m_driverController.rightBumper())
        .onTrue(new InstantCommand(() -> autoAlignRunning.setPressed(true)));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void periodic() {
    /*
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
    */
    superstructure.periodic();
  }
}
