// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public final class SwerveConstants {

  public static final double wheelRadiusMaxVelocity = 2 * Math.PI; // Rad/Sec
  public static final double wheelRadiusMaxRampRate = Math.PI / 2; // Rad/Sec^2

  public static final Rotation2d mod1XOffset = new Rotation2d(135);
  public static final Rotation2d mod2XOffset = new Rotation2d(45);
  public static final Rotation2d mod3XOffset = new Rotation2d(mod1XOffset.getDegrees() + 180);
  public static final Rotation2d mod4XOffset = new Rotation2d(mod2XOffset.getDegrees() + 180);

  public static final Rotation2d uniformHOffset = new Rotation2d(90);

  // source 1
  public static final Rotation2d sourceLeft1 = new Rotation2d(125.989);

  // source 2
  public static final Rotation2d sourceRight2 = new Rotation2d(-sourceLeft1.getDegrees());

  // climb
  public static final Rotation2d hang = new Rotation2d(0);

  // barge
  public static final Rotation2d barge = new Rotation2d(90);

  // angle backups
  public static final Rotation2d reefAB = new Rotation2d(0);
  public static final Rotation2d reefCD = new Rotation2d(60);
  public static final Rotation2d reefEF = new Rotation2d(120);
  public static final Rotation2d reefGH = new Rotation2d(180);
  public static final Rotation2d reefIJ = new Rotation2d(-reefCD.getDegrees());
  public static final Rotation2d reefKL = new Rotation2d(-reefEF.getDegrees());

  public static final double aziTimeout = 1;

  public static final Mass robotMass = Pounds.of(120);
  public static final MomentOfInertia robotMOI = KilogramSquareMeters.of(36);

  public static final Translation2d frontLeft =
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY);
  public static final Translation2d frontRight =
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY);
  public static final Translation2d backLeft =
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY);
  public static final Translation2d backRight =
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY);
  public static final RobotConfig ppRobotConfig =
      new RobotConfig(
          SwerveConstants.robotMass,
          SwerveConstants.robotMOI,
          new ModuleConfig(
              TunerConstants.kWheelRadius,
              TunerConstants.kSpeedAt12Volts,
              1,
              DCMotor.getKrakenX60(1),
              Amps.of(80),
              1),
          frontLeft,
          frontRight,
          backLeft,
          backRight);

  public static final AngularVelocity maxSteerModuleSpeed = RotationsPerSecond.of(1);

  public static final SwerveModuleState mod1XState = new SwerveModuleState(0.0, mod1XOffset);
  public static final SwerveModuleState mod2XState = new SwerveModuleState(0.0, mod2XOffset);
  public static final SwerveModuleState mod3XState = new SwerveModuleState(0.0, mod3XOffset);
  public static final SwerveModuleState mod4XState = new SwerveModuleState(0.0, mod4XOffset);

  public static final SwerveModuleState lockHorizontal = new SwerveModuleState(0.0, uniformHOffset);
}
