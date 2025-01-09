// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Transform2d;

public final class SwerveConstants {

  public static final Transform2d robotToQuest = new Transform2d();
  public static final double wheelRadiusMaxVelocity = 0.25; // Rad/Sec
  public static final double wheelRadiusMaxRampRate = 0.05; // Rad/Sec^2
}
