// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public final class SwerveConstants {

  public static final double wheelRadiusMaxVelocity = 0.25; // Rad/Sec
  public static final double wheelRadiusMaxRampRate = 0.05; // Rad/Sec^2

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

  public static final double AB1 = -0.5;
  public static final double AB2 = 0.5;
  public static final double CD1 = 0.0;
  public static final double CD2 = 0.0;
  public static final double EF1 = 0.0;
  public static final double EF2 = 0.0;
  public static final double GH1 = 0.0;
  public static final double GH2 = 0.0;
  public static final double IJ1 = 0.0;
  public static final double IJ2 = 0.0;
  public static final double KL1 = 0.0;
  public static final double KL2 = 0.0;
}
