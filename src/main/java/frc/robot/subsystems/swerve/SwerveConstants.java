// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public final class SwerveConstants {

  public static final double wheelRadiusMaxVelocity = 0.25; // Rad/Sec
  public static final double wheelRadiusMaxRampRate = 0.05; // Rad/Sec^2

  public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.magnitude();
  public static final double MaxAngularRate = 1.5 * Math.PI; // My drivetrain
  public static final double SlowMaxSpeed = MaxSpeed * 0.3;
  public static final double SlowMaxAngular = MaxAngularRate * 0.3;

  public static final double aziDrivekP = 0.0;
  public static final double aziDrivekI = 0.0;
  public static final double aziDrivekD = 0.0;

  public static final Rotation2d sourceLeft1 = new Rotation2d(125.989);
  // source 1
  public static final Rotation2d sourceRight2 = new Rotation2d(-sourceLeft1.getDegrees());
  // source2
  public static final Rotation2d hang = new Rotation2d(180);

  public static final Rotation2d reefAB = new Rotation2d(0);
  public static final Rotation2d reefCD = new Rotation2d(60);
  public static final Rotation2d reefEF = new Rotation2d(120);
  public static final Rotation2d reefGH = new Rotation2d(180);
  public static final Rotation2d reefIJ = new Rotation2d(-reefCD.getDegrees());
  public static final Rotation2d reefKL = new Rotation2d(-reefEF.getDegrees());

  public static final double aziTimeout = 0.3;
}
