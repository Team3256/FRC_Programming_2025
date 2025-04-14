// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils.autoaim;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/*
 * Adapted from 8033's Auto Aim implementation.
 * This auto aim implementation is for "last-mile" aligning, as "repulsor" or PathPlanner's OTF does not precisely align^*
 *
 * (*) Debated.
 */
public class AutoAim {

  public static final Translation2d BLUE_REEF_CENTER =
      new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
  public static final Translation2d RED_REEF_CENTER = ChoreoAllianceFlipUtil.flip(BLUE_REEF_CENTER);

  public static double BLUE_NET_X = 8.76 + Units.inchesToMeters(30.0);
  public static double RED_NET_X = ChoreoAllianceFlipUtil.flipX(BLUE_NET_X);

  public static final double TRANSLATION_TOLERANCE_METERS = Units.inchesToMeters(5.0);
  public static final double ROTATION_TOLERANCE_RADIANS = Units.degreesToRadians(2.0);

  public static boolean isInToleranceCoral(Pose2d pose) {
    final var diff = pose.minus(CoralTargets.getClosestTarget(pose));
    // System.out.println("AADiff: " + diff);
    // System.out.println("isNearX: " + MathUtil.isNear(0.0, diff.getX(),
    // AutoAim.TRANSLATION_TOLERANCE_METERS));
    // System.out.println("isNearY: " + MathUtil.isNear(0.0, diff.getY(),
    // AutoAim.TRANSLATION_TOLERANCE_METERS));
    // System.out.println("isNearRot: " + MathUtil.isNear(0.0,
    // diff.getRotation().getRadians(),
    // AutoAim.ROTATION_TOLERANCE_RADIANS));
    return MathUtil.isNear(
            0.0, Math.hypot(diff.getX(), diff.getY()), AutoAim.TRANSLATION_TOLERANCE_METERS)
        && MathUtil.isNear(
            0.0, diff.getRotation().getRadians(), AutoAim.ROTATION_TOLERANCE_RADIANS);
  }

  public static boolean isInToleranceAlgaeIntake(Pose2d pose) {
    final var diff = pose.minus(AlgaeIntakeTargets.getClosestTarget(pose));
    return MathUtil.isNear(
            0.0, Math.hypot(diff.getX(), diff.getY()), AutoAim.TRANSLATION_TOLERANCE_METERS)
        && MathUtil.isNear(
            0.0, diff.getRotation().getRadians(), AutoAim.ROTATION_TOLERANCE_RADIANS);
  }
}
