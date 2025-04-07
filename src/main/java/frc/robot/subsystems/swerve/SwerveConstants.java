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
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public final class SwerveConstants {

  // source 1, 5 degree overshoot for weight drag from elevator // + (2 * Math.PI)
  // overshoot if weight concentrated = (0.0872665)
  public static final Rotation2d sourceLeft1 = new Rotation2d(0.696);

  // source 2, 5 degree overshoot
  public static final Rotation2d sourceRight2 = new Rotation2d(2.527);

  // climb
  public static final Rotation2d hang = new Rotation2d(0);
  public static final Rotation2d hangBack = new Rotation2d(Math.PI);

  // barge
  public static final Rotation2d barge = new Rotation2d(Math.PI / 2);

  // angle backups
  public static final Rotation2d reefAB = Rotation2d.fromDegrees(90 - 5);
  public static final Rotation2d reefCD = Rotation2d.fromDegrees(149 - 5);
  public static final Rotation2d reefEF = Rotation2d.fromDegrees(-150 - 5);
  public static final Rotation2d reefGH = Rotation2d.fromDegrees(-90 - 5);
  public static final Rotation2d reefIJ = Rotation2d.fromDegrees(-30 - 5);
  public static final Rotation2d reefKL = Rotation2d.fromDegrees(30 - 5);

  public static final double aziTimeout = 1;
  public static final double aziTimeout2 = 1.2;

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
}
