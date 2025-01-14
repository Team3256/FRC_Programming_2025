// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.autogen;

import edu.wpi.first.units.measure.Time;

public record Node(
    NodeType nodeType,
    IntakeLocations intakeLocation,
    ScoringLocations scoringLocation,
    ScoringTypes scoringType,
    Time waitTime) {
  public Node {
    if ((intakeLocation == null || scoringLocation == null || scoringType == null)
        && (nodeType != NodeType.WAIT && nodeType != NodeType.DRIVE_AND_WAIT)) {
      throw new IllegalArgumentException(
          "Node must have intake, scoring, and scoring type if not a wait node");
    }
    if ((intakeLocation == null || scoringLocation == null)
        && nodeType == NodeType.DRIVE_AND_WAIT) {
      throw new IllegalArgumentException(
          "Drive and wait node must have intake and scoring location");
    }
    if ((nodeType == NodeType.WAIT || nodeType == NodeType.DRIVE_AND_WAIT) && waitTime == null) {
      throw new IllegalArgumentException("Wait node must have a wait time");
    }
  }

  // For wait node
  public Node(NodeType nodeType, Time waitTime) {
    this(nodeType, null, null, null, waitTime);
  }

  // FOr score node
  public Node(
      NodeType nodeType,
      IntakeLocations intakeLocation,
      ScoringLocations scoringLocation,
      ScoringTypes scoringType) {
    this(nodeType, intakeLocation, scoringLocation, scoringType, null);
  }

  public Node(
      NodeType nodeType,
      IntakeLocations intakeLocation,
      ScoringLocations scoringLocation,
      Time waitTime) {
    this(nodeType, intakeLocation, scoringLocation, null, waitTime);
  }
}
