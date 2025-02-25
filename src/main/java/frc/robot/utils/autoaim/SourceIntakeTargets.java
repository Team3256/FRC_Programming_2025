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
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public enum SourceIntakeTargets {
  // All coordinates are global coordinates from the lower, blue alliance side
  // corner, if the walls
  // were extended beyond the coral station
  // All angles from the center of the coral with 0° across the width of the
  // field, counterclockwise
  SOURCE_L_BLUE(new Pose2d(1.1705577373504639, 7.092771530151367, Rotation2d.fromRadians(2.21))),
  SOURCE_R_BLUE(new Pose2d(0.76, 1.26, Rotation2d.fromRadians(-2.21))),

  SOURCE_L_RED(ChoreoAllianceFlipUtil.flip(SOURCE_L_BLUE.location)),
  SOURCE_R_RED(ChoreoAllianceFlipUtil.flip(SOURCE_R_BLUE.location));

  public final Pose2d location;

  private SourceIntakeTargets(Pose2d location) {
    this.location = location;
  }

  private static final List<Pose2d> transformedPoses =
      Arrays.stream(values())
          .map(
              (SourceIntakeTargets targets) -> {
                return SourceIntakeTargets.getRobotTargetLocation(targets.location);
              })
          .toList();

  public static Pose2d getRobotTargetLocation(Pose2d original) {
    // source intake targets are already offset in terms of the bot
    return original;

    // return original.transformBy(
    // new Transform2d(
    // 0.4705 + Units.inchesToMeters(3.625) + 0.25,
    // 0,
    // Rotation2d.fromDegrees(180.0)));
  }

  /** Gets the closest offset target to the given pose. */
  public static Pose2d getClosestTarget(Pose2d pose) {
    return pose.nearest(transformedPoses);
  }

  public static SourceIntakeTargets getClosestTargetE(Pose2d pose, boolean leftHanded) {
    return Arrays.stream(values())
        // Lefthandedness is because each face of the reef has two branches going up
        // so A is left, B is right
        // .filter((target) -> target.leftHanded == leftHanded)
        .min(
            Comparator.comparingDouble(
                (SourceIntakeTargets target) ->
                    pose.getTranslation()
                        .getDistance(
                            SourceIntakeTargets.getRobotTargetLocation(target.location)
                                .getTranslation())))
        .get();
  }
}
