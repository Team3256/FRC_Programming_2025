// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils.autoaim;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public enum CoralTargets {
  // All coordinates are global coordinates from the lower, blue alliance side
  // corner, if the walls
  // were extended beyond the coral station
  // All angles from the center of the coral with 0° across the width of the
  // field, counterclockwise
  BLUE_A(new Pose2d(3.95, 4.20, Rotation2d.fromDegrees(90)), true),
  BLUE_B(new Pose2d(3.95, 3.87, Rotation2d.fromDegrees(90)), false),
  BLUE_C(new Pose2d(4.07, 3.66, Rotation2d.fromDegrees(150)), true),
  BLUE_D(new Pose2d(4.35, 3.49, Rotation2d.fromDegrees(150)), false),
  BLUE_E(new Pose2d(4.60, 3.50, Rotation2d.fromDegrees(210)), true),
  BLUE_F(new Pose2d(4.88, 3.66, Rotation2d.fromDegrees(210)), false),
  BLUE_G(new Pose2d(5.00, 3.90, Rotation2d.fromDegrees(270)), true),
  BLUE_H(new Pose2d(5.00, 4.20, Rotation2d.fromDegrees(270)), false),
  BLUE_I(new Pose2d(4.88, 4.41, Rotation2d.fromDegrees(330)), true),
  BLUE_J(new Pose2d(4.60, 4.57, Rotation2d.fromDegrees(330)), false),
  BLUE_K(new Pose2d(4.36, 4.57, Rotation2d.fromDegrees(30)), true),
  BLUE_L(new Pose2d(4.06, 4.41, Rotation2d.fromDegrees(30)), false),

  RED_A(ChoreoAllianceFlipUtil.flip(BLUE_A.location), true),
  RED_B(ChoreoAllianceFlipUtil.flip(BLUE_B.location), false),
  RED_C(ChoreoAllianceFlipUtil.flip(BLUE_C.location), true),
  RED_D(ChoreoAllianceFlipUtil.flip(BLUE_D.location), false),
  RED_E(ChoreoAllianceFlipUtil.flip(BLUE_E.location), false),
  RED_F(ChoreoAllianceFlipUtil.flip(BLUE_F.location), true),
  RED_G(ChoreoAllianceFlipUtil.flip(BLUE_G.location), false),
  RED_H(ChoreoAllianceFlipUtil.flip(BLUE_H.location), true),
  RED_I(ChoreoAllianceFlipUtil.flip(BLUE_I.location), false),
  RED_J(ChoreoAllianceFlipUtil.flip(BLUE_J.location), true),
  RED_K(ChoreoAllianceFlipUtil.flip(BLUE_K.location), true),
  RED_L(ChoreoAllianceFlipUtil.flip(BLUE_L.location), false);

  public final Pose2d location;
  public final boolean leftHanded;

  private CoralTargets(Pose2d location, boolean leftHanded) {
    this.location = location;
    this.leftHanded = leftHanded;
  }

  private static final List<Pose2d> transformedPoses =
      Arrays.stream(values())
          .map(
              (CoralTargets targets) -> {
                return CoralTargets.getRobotTargetLocation(targets.location);
              })
          .toList();

  public static Pose2d getRobotTargetLocation(Pose2d original) {
    // return original.transformBy();
    // 0.248 for trough
    // tested values
    // 0.291
    // 0.3955 ((0.291 + 0.5) / 2)
    // 0.4705 ((0.291 + 0.65) / 2)
    // 3.625 is bumper length
    // maybe add a bit more to account for robot not aligning perfectly
    return original.transformBy(
        new Transform2d(
            0.4705 + Units.inchesToMeters(3.625) + 0.25, // TODO: TUNE This!!
            0,
            Rotation2d.fromDegrees(180.0 + 90)));
  }

  /** Gets the closest offset target to the given pose. */
  public static Pose2d getClosestTarget(Pose2d pose) {
    return pose.nearest(transformedPoses);
  }

  /** Gets the closest offset target to the given pose. */
  public static Pose2d getHandedClosestTarget(Pose2d pose, boolean leftHanded) {
    return pose.nearest(
        Arrays.stream(values())
            // Lefthandedness is because each face of the reef has two branches going up
            // so A is left, B is right
            .filter((target) -> target.leftHanded == leftHanded)
            .map(
                (CoralTargets targets) -> {
                  return CoralTargets.getRobotTargetLocation(targets.location);
                })
            .toList());
  }

  public static CoralTargets getHandedClosestTargetE(Pose2d pose, boolean leftHanded) {
    return Arrays.stream(values())
        // Lefthandedness is because each face of the reef has two branches going up
        // so A is left, B is right
        .filter((target) -> target.leftHanded == leftHanded)
        .min(
            Comparator.comparingDouble(
                (CoralTargets target) ->
                    pose.getTranslation()
                        .getDistance(
                            CoralTargets.getRobotTargetLocation(target.location).getTranslation())))
        .get();
  }
}
