// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.warriorb.lib.drivers.XboxController;

public class AngleCalculator {
  public static Rotation2d getStickAngle(XboxController control) {
    return new Rotation2d(Math.atan2(control.getRightY(), control.getRightX()));
  }
}
