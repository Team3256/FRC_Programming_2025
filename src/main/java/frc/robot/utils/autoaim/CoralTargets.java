// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils.autoaim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public enum CoralTargets {
  // All coordinates are global coordinates from the lower, blue alliance side
  // corner, if the walls
  // were extended beyond the coral station
  // All angles from the center of the coral with 0° across the width of the
  // field, counterclockwise
  BLUE_A(new Pose2d(3.24, 4.20, Rotation2d.fromDegrees(90)), true), // did
  BLUE_B(new Pose2d(3.24, 3.865, Rotation2d.fromDegrees(90)), false), // did
  BLUE_C(new Pose2d(3.70, 3.02, Rotation2d.fromDegrees(150)), true), // did
  BLUE_D(new Pose2d(3.98, 2.86, Rotation2d.fromDegrees(150)), false), // did
  BLUE_E(new Pose2d(4.97, 2.85, Rotation2d.fromDegrees(-150)), true), // did
  BLUE_F(new Pose2d(5.25, 3.01, Rotation2d.fromDegrees(-150)), false), // did
  BLUE_G(new Pose2d(5.75, 3.85, Rotation2d.fromDegrees(-90)), true), // did
  BLUE_H(new Pose2d(5.75, 4.19, Rotation2d.fromDegrees(-90)), false), // did
  BLUE_I(new Pose2d(5.28, 5.03, Rotation2d.fromDegrees(-30)), true), //
  BLUE_J(new Pose2d(4.98, 5.20, Rotation2d.fromDegrees(-30)), false), // did
  BLUE_K(new Pose2d(4.03, 5.22, Rotation2d.fromDegrees(30)), true), // did
  BLUE_L(new Pose2d(3.70, 5.02, Rotation2d.fromDegrees(30)), false), // did

  //  RED_A(new Pose2d(3.22, 4.20, Rotation2d.fromDegrees(90)), true),
  //  RED_B(new Pose2d(3.18, 3.85, Rotation2d.fromDegrees(90)), false),
  //  RED_C(new Pose2d(3.70, 3.00, Rotation2d.fromDegrees(148)), true),
  RED_D(new Pose2d(13.54, 5.20, Rotation2d.fromDegrees(-30)), false),
  //  //  RED_E(new Pose2d(4.95, 2.83, Rotation2d.fromDegrees(-150)), true),
  //  RED_F(new Pose2d(5.26, 2.95, Rotation2d.fromDegrees(-150)), false),
  RED_G(new Pose2d(11.80, 4.21, Rotation2d.fromDegrees(90)), true),
  RED_H(new Pose2d(11.80, 3.87, Rotation2d.fromDegrees(90)), false),
  RED_I(new Pose2d(12.28, 3.02, Rotation2d.fromDegrees(150)), true), // did
  //  RED_J(new Pose2d(12.37, 5.05, Rotation2d.fromDegrees(30)), false),
  //  RED_K(new Pose2d(13.55, 2.84, Rotation2d.fromDegrees(-150)), true),
  //  RED_L(new Pose2d(12.27, 3.02, Rotation2d.fromDegrees(150)), false),
  RED_A(new Pose2d(14.33, 3.85, Rotation2d.fromDegrees(-90)), true),
  RED_B(new Pose2d(14.32, 4.20, Rotation2d.fromDegrees(-90)), false),
  RED_C(new Pose2d(13.84, 5.03, Rotation2d.fromDegrees(-30)), true),
  //  //  RED_D(ChoreoAllianceFlipUtil.flip(BLUE_D.location), false),
  RED_E(new Pose2d(12.60, 5.20, Rotation2d.fromDegrees(30)), false),
  RED_F(new Pose2d(12.30, 5.03, Rotation2d.fromDegrees(30)), true),
  //  RED_G(ChoreoAllianceFlipUtil.flip(BLUE_G.location), false),
  //  RED_H(ChoreoAllianceFlipUtil.flip(BLUE_H.location), true),
  //  RED_I(ChoreoAllianceFlipUtil.flip(BLUE_I.location), false),
  RED_J(new Pose2d(12.57, 2.86, Rotation2d.fromDegrees(150)), true), // did
  RED_K(new Pose2d(13.52, 2.84, Rotation2d.fromDegrees(-150)), true), // did
  RED_L(new Pose2d(13.83, 3.02, Rotation2d.fromDegrees(-150)), false); // did

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
            0, // TODO: TUNE This!!
            0,
            Rotation2d.kZero));
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
