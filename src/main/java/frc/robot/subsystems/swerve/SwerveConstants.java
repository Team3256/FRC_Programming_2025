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
  public static final Rotation2d sourceLeft1 = new Rotation2d(0.696 - (2 * 0.087266));

  // source 2, 5 degree overshoot
  public static final Rotation2d sourceRight2 = new Rotation2d(2.527 - (2 * 0.0872665));

  // climb
  public static final Rotation2d hang = new Rotation2d(Math.PI);

  // barge
  public static final Rotation2d barge = new Rotation2d(-Math.PI / 2);

  // angle backups
  public static final Rotation2d reefAB = new Rotation2d(0 + (Math.PI / 2) - (2 * 0.0872665));
  public static final Rotation2d reefCD = new Rotation2d((Math.PI / 3) + (Math.PI / 2) - (2 * 0.0872665));
  public static final Rotation2d reefEF = new Rotation2d((2 * reefCD.getRadians()) + (Math.PI / 2) - (2 * 0.0872665));
  public static final Rotation2d reefGH = new Rotation2d(Math.PI + (Math.PI / 2) - (2 * 0.0872665));
  public static final Rotation2d reefIJ = new Rotation2d(-reefCD.getRadians());
  public static final Rotation2d reefKL = new Rotation2d(-reefEF.getRadians());

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
