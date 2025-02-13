// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.drivers.QuestNav;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.subsystems.swerve.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utils.RepulsorFieldPlanner;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants.FeatureFlags;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

  public static final double driveBaseRadius =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  @AutoLogOutput private double lastGyroYawRads = 0;
  @AutoLogOutput private double accumGyroYawRads = 0;
  @AutoLogOutput private double averageWheelPosition = 0;
  @AutoLogOutput private double[] startWheelPositions = new double[4];
  @AutoLogOutput private double currentEffectiveWheelRadius = 0;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /** Swerve request to apply during field-centric path following */
  private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  private final PIDController m_pathXController = new PIDController(25, 0, 0);
  private final PIDController m_pathYController = new PIDController(25, 0, 0);
  private final PIDController m_pathThetaController = new PIDController(6, 0, 0);

  private final PIDController xController = new PIDController(5.0, 0.0, 0.1);
  private final PIDController yController = new PIDController(5.0, 0.0, 0.1);
  private final PIDController headingController = new PIDController(6, 0, 0);

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  private final SwerveSetpointGenerator setpointGenerator =
      new SwerveSetpointGenerator(
          SwerveConstants.ppRobotConfig, SwerveConstants.maxSteerModuleSpeed);
  private SwerveSetpoint previousSetpoint =
      new SwerveSetpoint(
          new ChassisSpeeds(0, 0, 0),
          new SwerveModuleState[] {
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0))
          },
          new DriveFeedforwards(
              new double[] {0},
              new double[] {0},
              new double[] {0},
              new double[] {0},
              new double[] {0}));

  /*
   * SysId routine for characterizing translation. This is used to find PID gains
   * for the drive motors.
   */
  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /*
   * SysId routine for characterizing steer. This is used to find PID gains for
   * the steer motors.
   */
  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle
   * HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
   * importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  private final QuestNav questNav = new QuestNav();

  public RepulsorFieldPlanner m_repulsor = new RepulsorFieldPlanner();

  private Translation2d _calculatedOffsetToRobotCenter = new Translation2d();
  private int _calculatedOffsetToRobotCenterCount = 0;

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          SwerveConstants.frontLeft,
          SwerveConstants.frontRight,
          SwerveConstants.backLeft,
          SwerveConstants.backRight);
  /* WPILib Alerts start */

  private final Alert a_questNavNotConnected =
      new Alert("QuestNav failure (no data within 250ms)", AlertType.kError);

  /* WPILib Alerts end */
  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configurePathPlanner();
    // resetPoseAndQuest(new Pose2d());
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configurePathPlanner();
    // resetPoseAndQuest(new Pose2d());
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   *     [x, y, theta]ᵀ, with units in meters and radians
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta]ᵀ, with units in meters and radians
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configurePathPlanner();
    // resetPoseAndQuest(new Pose2d());
  }

  public Pose2d targetPose() {
    return new Pose2d(
        m_pathXController.getSetpoint(),
        m_pathYController.getSetpoint(),
        Rotation2d.fromRadians(m_pathThetaController.getSetpoint()));
  }

  public Command repulsorCommand(Supplier<Pose2d> target) {
    if (!FeatureFlags.kAutoAlignEnabled) {
      System.out.println("**** repulsorCommand disabled because of kAutoAlignEnabled = false");
      return Commands.none();
    }
    return run(
        () -> {
          m_repulsor.setGoal(target.get().getTranslation());
          if (this.getState().Pose.getTranslation().getDistance(target.get().getTranslation())
              < .3) {
            this.setControl(
                m_pathApplyFieldSpeeds.withSpeeds(
                    new ChassisSpeeds(
                        xController.calculate(this.getState().Pose.getX(), target.get().getX()),
                        yController.calculate(this.getState().Pose.getY(), target.get().getY()),
                        headingController.calculate(
                            this.getState().Pose.getRotation().getRadians(),
                            target.get().getRotation().getRadians()))));
          } else {
            followPathRepulsor(
                this.getState().Pose,
                m_repulsor.getCmd(
                    this.getState().Pose,
                    this.getState().Speeds,
                    4.2,
                    true,
                    target.get().getRotation()));
          }
        });
  }

  /**
   * Creates a new auto factory for this drivetrain.
   *
   * @return AutoFactory for this drivetrain
   */
  public AutoFactory createAutoFactory() {
    return createAutoFactory((sample, isStart) -> {});
  }

  public void trajLogger(Trajectory<SwerveSample> sample, boolean isStart) {
    Logger.recordOutput(this.getClass().getSimpleName() + "/Choreo/TrajPoses", sample.getPoses());
  }

  /**
   * Creates a new auto factory for this drivetrain with the given trajectory logger.
   *
   * @param trajLogger Logger for the trajectory
   * @return AutoFactory for this drivetrain
   */
  public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
    return new AutoFactory(
        () -> getState().Pose, this::resetPoseAndQuest, this::followPath, true, this, trajLogger);
  }

  public void resetPoseAndQuest(Pose2d pose) {
    super.resetPose(pose);
    questNav.resetPose(pose);
  }

  public Command determineOffsetToRobotCenter() {
    return Commands.repeatingSequence(
            this.runOnce(
                    () -> {
                      this.setControl(
                          new SwerveRequest.ApplyRobotSpeeds()
                              .withSpeeds(new ChassisSpeeds(0, 0, .314)));
                    })
                .withTimeout(2.0),
            Commands.runOnce(
                () -> {
                  // Update current offset
                  Translation2d offset =
                      questNav.calculateOffsetToRobotCenter(this.getState().Pose);

                  _calculatedOffsetToRobotCenter =
                      _calculatedOffsetToRobotCenter
                          .times(
                              (double) _calculatedOffsetToRobotCenterCount
                                  / (_calculatedOffsetToRobotCenterCount + 1))
                          .plus(offset.div(_calculatedOffsetToRobotCenterCount + 1));
                  _calculatedOffsetToRobotCenterCount++;

                  SmartDashboard.putNumberArray(
                      "Quest Calculated Offset to Robot Center",
                      new double[] {
                        _calculatedOffsetToRobotCenter.getX(), _calculatedOffsetToRobotCenter.getY()
                      });
                }))
        .withTimeout(10.0);
  }

  public void followPathRepulsor(Pose2d pose, SwerveSample sample) {
    m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

    var targetSpeeds = sample.getChassisSpeeds();
    targetSpeeds.vyMetersPerSecond = m_pathXController.calculate(pose.getX(), sample.x);
    targetSpeeds.vyMetersPerSecond = m_pathYController.calculate(pose.getY(), sample.y);
    targetSpeeds.omegaRadiansPerSecond +=
        m_pathThetaController.calculate(pose.getRotation().getRadians(), sample.heading);
    this.setControl(
        m_pathApplyFieldSpeeds
            .withSpeeds(targetSpeeds)
            .withWheelForceFeedforwardsX(sample.moduleForcesX())
            .withWheelForceFeedforwardsY(sample.moduleForcesY()));
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param requestSupplier Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public Command applyRequest(SwerveRequest swerveRequest) {
    return run(() -> this.setControl(swerveRequest));
  }

  /**
   * Follows the given field-centric path sample with PID.
   *
   * @param sample Sample along the path to follow
   */
  public void followPath(SwerveSample sample) {
    m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

    var pose = getState().Pose;

    var targetSpeeds = sample.getChassisSpeeds();
    targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(pose.getX(), sample.x);
    targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(pose.getY(), sample.y);
    targetSpeeds.omegaRadiansPerSecond +=
        m_pathThetaController.calculate(pose.getRotation().getRadians(), sample.heading);

    setControl(
        m_pathApplyFieldSpeeds
            .withSpeeds(targetSpeeds)
            .withWheelForceFeedforwardsX(sample.moduleForcesX())
            .withWheelForceFeedforwardsY(sample.moduleForcesY()));
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  public Command sysIdRotationQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRotation.quasistatic(direction);
  }

  public Command sysIdRotationDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRotation.dynamic(direction);
  }

  public Command sysIdTranslationQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineTranslation.quasistatic(direction);
  }

  public Command sysIdTranslationDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineTranslation.dynamic(direction);
  }

  public Command driveVelocityFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.applyRequest(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(speeds.get()));
  }

  public ChassisSpeeds getVelocityFieldRelative() {
    return this.getState().Speeds;
  }

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply
     * it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is
     * disabled.
     * This ensures driving behavior doesn't change until an explicit disable event
     * occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }
    if (!Utils.isSimulation() && questNav.isConnected()) {
      // this.addVisionMeasurement(
      // questNav.getPose().transformBy(SwerveConstants.robotToQuest.inverse()),
      // Utils.getCurrentTimeSeconds());
      Logger.recordOutput("QuestNav/pose", questNav.getRobotPose());
      Logger.recordOutput("QuestNav/quaternion", questNav.getQuaternion());
      Logger.recordOutput("QuestNav/batteryPercent", questNav.getBatteryPercent());
    } else {
      a_questNavNotConnected.set(true);
    }

    Logger.recordOutput("Swerve/pose", this.getState().Pose);
  }

  private void configurePathPlanner() {
    AutoBuilder.configure(
        () -> this.getState().Pose,
        this::resetPoseAndQuest,
        () -> this.getState().Speeds,
        this::driveRobotRelativeWithFF,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        SwerveConstants.ppRobotConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  public void driveRobotRelativeWithFF(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    this.setControl(
        new SwerveRequest.ApplyRobotSpeeds()
            .withSpeeds(speeds)
            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons()));
  }

  public void driveFieldRelative(double xVelocity, double yVelocity, double angularVelocity) {

    previousSetpoint =
        setpointGenerator.generateSetpoint(
            previousSetpoint, new ChassisSpeeds(xVelocity, yVelocity, angularVelocity), 0.02);
    this.setControl(
        new SwerveRequest.ApplyFieldSpeeds()
            .withSpeeds(this.getKinematics().toChassisSpeeds(previousSetpoint.moduleStates())));
  }

  /**
   * Runs the wheel radius characterization routine.
   *
   * @param omegaDirection Direction of the robot to turn either 1 or -1
   */
  public Command wheelRadiusCharacterization(double omegaDirection) {

    /* wheel radius characterization schtuffs */
    final DoubleSupplier m_gyroYawRadsSupplier =
        () -> Units.degreesToRadians(getPigeon2().getYaw().getValueAsDouble());
    // () -> getState().Pose.getRotation().getRadians();
    final SlewRateLimiter m_omegaLimiter = new SlewRateLimiter(0.5);
    final SwerveRequest.RobotCentric m_characterizationReq =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    final double m_characterizationSpeed = 1.5;

    var initialize =
        runOnce(
            () -> {
              lastGyroYawRads = m_gyroYawRadsSupplier.getAsDouble();
              accumGyroYawRads = 0;
              currentEffectiveWheelRadius = 0;
              averageWheelPosition = 0;
              for (int i = 0; i < getModules().length; i++) {
                var pos = getModules()[i].getPosition(true);
                startWheelPositions[i] =
                    pos.distanceMeters * TunerConstants.kDriveRotationsPerMeter;
              }
              m_omegaLimiter.reset(0);
            });

    var executeEnd =
        runEnd(
            () -> {
              setControl(
                  m_characterizationReq.withRotationalRate(
                      m_omegaLimiter.calculate(m_characterizationSpeed * omegaDirection)));
              accumGyroYawRads +=
                  MathUtil.angleModulus(m_gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
              lastGyroYawRads = m_gyroYawRadsSupplier.getAsDouble();
              averageWheelPosition = 0;
              double[] wheelPositions = new double[4];
              for (int i = 0; i < getModules().length; i++) {
                var pos = getModules()[i].getPosition(true);
                wheelPositions[i] = pos.distanceMeters * TunerConstants.kDriveRotationsPerMeter;
                averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
              }
              averageWheelPosition = averageWheelPosition / 4.0;
              currentEffectiveWheelRadius =
                  (accumGyroYawRads * TunerConstants.kDriveRadius) / averageWheelPosition;
              // System.out.println("effective wheel radius: " + currentEffectiveWheelRadius);
              System.out.println("Average Wheel Position: " + averageWheelPosition);
            },
            () -> {
              setControl(m_characterizationReq.withRotationalRate(0));
              if (Math.abs(accumGyroYawRads) <= Math.PI * 2.0) {
                System.out.println(
                    "not enough data for characterization "
                        + accumGyroYawRads
                        + "\navgWheelPos: "
                        + averageWheelPosition
                        + "radians");
              } else {
                System.out.println(
                    "effective wheel radius: "
                        + currentEffectiveWheelRadius
                        + " inches"
                        + "\naccumGryoYawRads: "
                        + accumGyroYawRads
                        + "radians"
                        + "\navgWheelPos: "
                        + averageWheelPosition
                        + "radians");
              }
            });

    return Commands.sequence(initialize, executeEnd);
  }
}
