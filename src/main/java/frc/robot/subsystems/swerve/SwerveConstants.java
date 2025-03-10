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

  public static final Rotation2d uniformHLockOffset = new Rotation2d(0);

  // source 1
  public static final Rotation2d sourceLeft1 = new Rotation2d(0.749 - 0.0872665);

  // source 2
  public static final Rotation2d sourceRight2 = new Rotation2d(2.621 - 0.0872665);

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
}
