// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.autogen;

import static edu.wpi.first.units.Units.Rotations;

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
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.ArrayList;

public class NodeManager {

  private final CommandSwerveDrivetrain drivetrain;
  private final Elevator elevator;
  private final Arm arm;
  private final EndEffector endEffector;

  private final AutoFactory factory;

  public NodeManager(
      CommandSwerveDrivetrain drivetrain,
      Elevator elevator,
      Arm arm,
      EndEffector endEffector,
      AutoFactory factory) {
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
          preloadTraj
              .atTimeBeforeEnd(.5)
              .toggleOnTrue(elevator.toReefLevel(3).alongWith(arm.toReefLevel(2, () -> true)));
          Command scoreCmd =
              Commands.waitUntil(elevator.reachedPosition.and(arm.reachedPosition))
                  .andThen(
                      endEffector
                          .setL4Voltage(() -> true)
                          .until(routine.observe(endEffector.coralBeamBreak))
                          .andThen(arm.toHome().alongWith(elevator.toHome())));
          preloadTraj.done().toggleOnTrue(scoreCmd);
          nextTrajTrigger = new Trigger(scoreCmd::isFinished);
          lastScoringLocation = node.scoringLocation();
        }
        case SCORE_AND_INTAKE -> {
          // Load intake traj
          AutoTrajectory intakeTraj =
              routine.trajectory(lastScoringLocation.name() + "-" + node.intakeLocation().name());
          // Wait for whatever finished last to be done then trigger next traj
          nextTrajTrigger.toggleOnTrue(intakeTraj.cmd());
          intakeTraj
              .atTimeBeforeEnd(1)
              .toggleOnTrue(elevator.setPosition(ElevatorConstants.sourcePosition.in(Rotations)));
          intakeTraj.atTimeBeforeEnd(.5).toggleOnTrue(arm.toSourceLevel());
          Command intakeCmd =
              endEffector
                  .setSourceVelocity()
                  .until(endEffector.coralBeamBreak.debounce(.1))
                  .andThen(
                      arm.toHome().alongWith(Commands.waitSeconds(.2).andThen(elevator.toHome())));
          intakeTraj.done().onTrue(intakeCmd);
          // Load scoring traj
          AutoTrajectory scoringTraj =
              routine.trajectory(
                  node.intakeLocation().name() + "-" + node.scoringLocation().name());
          new Trigger(intakeCmd::isFinished).toggleOnTrue(scoringTraj.cmd());
          // Wait for intake traj to be done then trigger scoring traj
          Command scoreCmd = Commands.none();
          switch (node.scoringType()) {
            case L1 -> {
              scoringTraj
                  .atTimeBeforeEnd(.5)
                  .toggleOnTrue(arm.toReefLevel(0, () -> true).alongWith(elevator.toReefLevel(0)));
              scoreCmd =
                  endEffector
                      .setL1Velocity(() -> true)
                      .until(endEffector.coralBeamBreak)
                      .andThen(arm.toHome().alongWith(elevator.toHome()));
              scoringTraj.done().onTrue(scoreCmd);
            }
            case L2 -> {
              scoringTraj
                  .atTimeBeforeEnd(.5)
                  .toggleOnTrue(arm.toReefLevel(1, () -> true).alongWith(elevator.toReefLevel(1)));
              scoreCmd =
                  endEffector
                      .setL2L3Velocity(() -> true)
                      .until(endEffector.coralBeamBreak)
                      .andThen(arm.toHome().alongWith(elevator.toHome()));
              scoringTraj.done().onTrue(scoreCmd);
            }
            case L3 -> {
              scoringTraj
                  .atTimeBeforeEnd(.5)
                  .toggleOnTrue(arm.toReefLevel(1, () -> true).alongWith(elevator.toReefLevel(2)));
              scoreCmd =
                  endEffector
                      .setL2L3Velocity(() -> true)
                      .until(endEffector.coralBeamBreak)
                      .andThen(arm.toHome().alongWith(elevator.toHome()));
              scoringTraj.done().onTrue(scoreCmd);
            }
            case L4 -> {
              scoringTraj
                  .atTimeBeforeEnd(.5)
                  .toggleOnTrue(arm.toReefLevel(2, () -> true).alongWith(elevator.toReefLevel(3)));
              scoreCmd =
                  endEffector
                      .setL4Voltage(() -> true)
                      .until(endEffector.coralBeamBreak)
                      .andThen(arm.toHome().alongWith(elevator.toHome()));
              scoringTraj.done().onTrue(scoreCmd);
            }
          }

          // Update last scoring location and trigger for next traj
          lastScoringLocation = node.scoringLocation();
          nextTrajTrigger = new Trigger(scoreCmd::isFinished);
        }
        case WAIT -> {
          Command waitCmd = Commands.waitTime(node.waitTime());
          nextTrajTrigger.toggleOnTrue(waitCmd);
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
