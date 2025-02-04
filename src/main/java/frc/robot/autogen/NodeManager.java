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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.rollers.Roller;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.ArrayList;

import static edu.wpi.first.units.Units.Rotations;

public class NodeManager {

  private final CommandSwerveDrivetrain drivetrain;
  private final Elevator elevator;
    private final Arm arm;
    private final EndEffector endEffector;

  private final AutoFactory factory;

  public NodeManager(CommandSwerveDrivetrain drivetrain, Elevator elevator, Arm arm, EndEffector endEffector, AutoFactory factory) {
    this.drivetrain = drivetrain;
    this.factory = factory;
    this.elevator = elevator;
    this.arm = arm;
    this.endEffector = endEffector;
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
          nextTrajTrigger.toggleOnTrue(preloadTraj.resetOdometry().andThen(preloadTraj.cmd()));
          TrajTriggers.atTimeToEnd(preloadTraj, .5)
              .toggleOnTrue(elevator.toReefLevel(3).alongWith(arm.toRightReefLevel(2)));
          preloadTraj.done().and(routine.observe(elevator.reachedPosition)).and(routine.observe(arm.reachedPosition)).toggleOnTrue(endEffector.setL4Velocity());
          endEffector.rightBeamBreak.negate().toggleOnTrue(arm.toHome()).toggleOnTrue(elevator.toHome());
          nextTrajTrigger = preloadTraj.done(60);
          lastScoringLocation = node.scoringLocation();
        }
        case SCORE_AND_INTAKE -> {
          // Load intake traj
          AutoTrajectory intakeTraj =
              routine.trajectory(lastScoringLocation.name() + "-" + node.intakeLocation().name());
          // Wait for whatever finished last to be done then trigger next traj
          nextTrajTrigger.toggleOnTrue(intakeTraj.cmd());
          TrajTriggers.atTimeToEnd(intakeTraj,1)
                          .toggleOnTrue(elevator.setPosition(ElevatorConstants.sourcePosition.in(Rotations)));
          TrajTriggers.atTimeToEnd(intakeTraj,.5).toggleOnTrue(arm.toRightSourceLevel());
          intakeTraj.done().toggleOnTrue(rollers.setRollerVoltage(3));
          // Load scoring traj
          AutoTrajectory scoringTraj =
              routine.trajectory(
                  node.intakeLocation().name() + "-" + node.scoringLocation().name());
          // Wait for intake traj to be done then trigger scoring traj
          switch (node.scoringType()) {
            case L1 -> {
              intakeTraj.done().toggleOnTrue(scoringTraj.cmd());
              scoringTraj.done().toggleOnTrue(rollers.setRollerVoltage(-3));
            }
            case L2 -> {
              intakeTraj.done().toggleOnTrue(scoringTraj.cmd());
              scoringTraj.done().toggleOnTrue(rollers.setRollerVoltage(-3));
            }
            case L3 -> {
              intakeTraj.done().toggleOnTrue(scoringTraj.cmd());
              scoringTraj.done().toggleOnTrue(rollers.setRollerVoltage(-3));
            }
            case L4 -> {
              intakeTraj.done().toggleOnTrue(scoringTraj.cmd());
              scoringTraj.done().toggleOnTrue(rollers.setRollerVoltage(-3));
            }
          }

          // Update last scoring location and trigger for next traj
          lastScoringLocation = node.scoringLocation();
          nextTrajTrigger = scoringTraj.done();
        }
        case WAIT -> {
          Command waitCmd = Commands.waitTime(node.waitTime());
          nextTrajTrigger.onTrue(waitCmd);
          nextTrajTrigger = routine.observe(waitCmd::isFinished);
        }

        case DRIVE_AND_WAIT -> {
          AutoTrajectory driveTraj =
              routine.trajectory(
                  node.intakeLocation().name() + "-" + node.scoringLocation().name());
          nextTrajTrigger.toggleOnTrue(driveTraj.cmd());
          Command waitCmd = Commands.waitTime(node.waitTime());
          driveTraj.done().onTrue(waitCmd);
          nextTrajTrigger = routine.observe(waitCmd::isFinished);
          lastScoringLocation = node.scoringLocation();
        }
      }
    }
    return routine;
  }
}
