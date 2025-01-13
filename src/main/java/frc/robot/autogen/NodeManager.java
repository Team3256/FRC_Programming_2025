// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.autogen;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.rollers.Roller;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.ArrayList;

public class NodeManager {

  private final CommandSwerveDrivetrain drivetrain;
  private final Roller rollers;

  private final AutoFactory factory;

  public NodeManager(CommandSwerveDrivetrain drivetrain, Roller rollers, AutoFactory factory) {
    this.drivetrain = drivetrain;
    this.rollers = rollers;
    this.factory = factory;
  }

  public AutoRoutine createAuto(ArrayList<Node> nodes) {
    AutoRoutine routine = factory.newRoutine("Auto");
    Trigger nextTrajTrigger = routine.active();
    ScoringLocations lastScoringLocation = null;
    for (Node node : nodes) {

      switch (node.nodeType()) {
        case PRELOAD -> {
          AutoTrajectory preloadTraj =
              routine.trajectory(
                  node.intakeLocation().name() + "-" + node.scoringLocation().name());
          nextTrajTrigger.onTrue(preloadTraj.resetOdometry().andThen(preloadTraj.cmd()));
          preloadTraj.done().onTrue(rollers.setRollerVoltage(3));
          nextTrajTrigger = preloadTraj.done(60);
          lastScoringLocation = node.scoringLocation();
        }
        case SCORE_AND_INTAKE -> {
          // Load intake traj
          AutoTrajectory intakeTraj =
              routine.trajectory(lastScoringLocation.name() + "-" + node.intakeLocation().name());
          // Wait for whatever finished last to be done then trigger next traj
          nextTrajTrigger.onTrue(intakeTraj.cmd());
          intakeTraj.done().onTrue(rollers.setRollerVoltage(3));

          // Load scoring traj
          AutoTrajectory scoringTraj =
              routine.trajectory(
                  node.intakeLocation().name() + "-" + node.scoringLocation().name());
          // Wait for intake traj to be done then trigger scoring traj
          intakeTraj.done().onTrue(scoringTraj.cmd());
          scoringTraj.done().onTrue(rollers.setRollerVoltage(-3));

          // Update last scoring location and trigger for next traj
          lastScoringLocation = node.scoringLocation();
          nextTrajTrigger = scoringTraj.done();
        }
      }
    }
    return routine;
  }
}
