// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.utils.autoaim.CoralTargets;
import frc.robot.utils.autoaim.SourceIntakeTargets;

public class AutoRoutines {
  private final AutoFactory m_factory;

  private final AutoCommands m_autoCommands;

  private final Elevator m_elevator;
  private final CommandSwerveDrivetrain m_drivetrain;
  private final Arm m_arm;
  private final EndEffector m_endEffector;

  public AutoRoutines(
      AutoFactory factory,
      Elevator elevator,
      Arm arm,
      EndEffector endEffector,
      CommandSwerveDrivetrain drivetrain) {
    m_factory = factory;
    m_elevator = elevator;
    m_arm = arm;
    m_drivetrain = drivetrain;
    m_endEffector = endEffector;
    m_autoCommands = new AutoCommands(elevator, arm, endEffector);
  }

  public AutoRoutine simplePathAuto() {
    final AutoRoutine routine = m_factory.newRoutine("funny");
    final AutoTrajectory simplePath = routine.trajectory("funny");

    routine.active().onTrue(simplePath.resetOdometry().andThen(simplePath.cmd()));
    return routine;
  }

  public AutoRoutine mobilityTop() {
    final AutoRoutine routine = m_factory.newRoutine("mobilityTop");
    final AutoTrajectory mobilityTop = routine.trajectory("MobilityTop");
    routine.active().onTrue(mobilityTop.resetOdometry().andThen(mobilityTop.cmd()));
    return routine;
  }

  public AutoRoutine mobilityBottom() {
    final AutoRoutine routine = m_factory.newRoutine("mobilityBottom");
    final AutoTrajectory mobilityBottom = routine.trajectory("MobilityBottom");
    routine.active().onTrue(mobilityBottom.resetOdometry().andThen(mobilityBottom.cmd()));
    return routine;
  }

  public AutoRoutine l4PreloadH() {
    final AutoRoutine routine = m_factory.newRoutine("l4PreloadH");
    final AutoTrajectory preloadH = routine.trajectory("MID-H");

    routine
        .active()
        .onTrue(preloadH.resetOdometry().andThen(Commands.waitSeconds(5)).andThen(preloadH.cmd()));
    preloadH.atTimeBeforeEnd(.5).onTrue(m_autoCommands.goToL4());
    preloadH
        .done()
        .onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
                .andThen(m_autoCommands.scoreL4())
                .until(m_endEffector.coralBeamBreak.negate().debounce(.5))
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () -> preloadH.getFinalPose().orElse(CoralTargets.BLUE_H.location)))
                .andThen(m_autoCommands.home()));
    //    l4Preload.atTimeBeforeEnd(.5).onTrue(m_autoCommands.goToL4());
    //    l4Preload
    //        .done()
    //        .onTrue(
    //
    // Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
    //                .andThen(m_autoCommands.scoreL4())
    //                .andThen(
    //                    Commands.waitUntil(
    //                        m_endEffector
    //                            .leftBeamBreak
    //                            .negate()
    //                            .and(m_endEffector.rightBeamBreak.negate())))
    //                .andThen(m_autoCommands.home()));

    return routine;
  }

  public AutoRoutine l4PreloadG() {
    final AutoRoutine routine = m_factory.newRoutine("l4PreloadG");
    final AutoTrajectory preloadG = routine.trajectory("MID-G");

    routine
        .active()
        .onTrue(preloadG.resetOdometry().andThen(Commands.waitSeconds(5)).andThen(preloadG.cmd()));
    preloadG.atTimeBeforeEnd(.5).onTrue(m_autoCommands.goToL4());
    preloadG
        .done()
        .onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
                .andThen(m_autoCommands.scoreL4())
                .until(m_endEffector.coralBeamBreak.negate().debounce(.5))
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () -> preloadG.getFinalPose().orElse(CoralTargets.BLUE_G.location)))
                .andThen(m_autoCommands.home()));
    //    l4Preload.atTimeBeforeEnd(.5).onTrue(m_autoCommands.goToL4());
    //    l4Preload
    //        .done()
    //        .onTrue(
    //
    // Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
    //                .andThen(m_autoCommands.scoreL4())
    //                .andThen(
    //                    Commands.waitUntil(
    //                        m_endEffector
    //                            .leftBeamBreak
    //                            .negate()
    //                            .and(m_endEffector.rightBeamBreak.negate())))
    //                .andThen(m_autoCommands.home()));

    return routine;
  }

  public AutoRoutine test() {
    final AutoRoutine routine = m_factory.newRoutine("l4CenterPreloadRightSource2");
    final AutoTrajectory preloadH = routine.trajectory("MID-H");
    final AutoTrajectory HtoSource = routine.trajectory("H-Source2");
    final AutoTrajectory SourceToC = routine.trajectory("Source2-C");
    final AutoTrajectory CtoSource = routine.trajectory("C-Source2");
    final AutoTrajectory SourceToD = routine.trajectory("Source2-D");

    routine.active().onTrue(preloadH.resetOdometry().andThen(preloadH.cmd()));
    preloadH.done().onTrue(HtoSource.spawnCmd());
    HtoSource.done().onTrue(SourceToC.spawnCmd());
    SourceToC.done().onTrue(CtoSource.spawnCmd());
    CtoSource.done().onTrue(SourceToD.spawnCmd());
    SourceToD.done().onTrue(m_autoCommands.goToL4());

    return routine;
  }

  public AutoRoutine l4CenterPreloadRightSource1() {
    final AutoRoutine routine = m_factory.newRoutine("l4CenterPreloadRightSource1");
    final AutoTrajectory preloadH = routine.trajectory("MID-H");
    final AutoTrajectory HtoSource = routine.trajectory("H-Source2");
    final AutoTrajectory SourceToC = routine.trajectory("Source2-C");

    routine.active().onTrue(preloadH.resetOdometry().andThen(preloadH.cmd()));
    preloadH.atTimeBeforeEnd(.5).onTrue(m_autoCommands.goToL4());
    preloadH
        .done()
        .onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
                .andThen(m_autoCommands.scoreL4())
                .until(m_endEffector.coralBeamBreak.negate())
                .andThen(
                    m_autoCommands
                        .home()
                        .alongWith(Commands.waitSeconds(.5).andThen(HtoSource.spawnCmd()))));

    HtoSource.atTimeBeforeEnd(.5)
        .onTrue(
            m_autoCommands
                .goToSource()
                .until(m_endEffector.coralBeamBreak)
                .andThen(m_autoCommands.homeSource().alongWith(SourceToC.spawnCmd())));
    SourceToC.atTimeBeforeEnd(.5).onTrue(m_autoCommands.goToL4());
    SourceToC.done()
        .onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
                .andThen(m_autoCommands.scoreL4())
                .until(m_endEffector.coralBeamBreak.negate())
                .andThen(m_autoCommands.home()));

    return routine;
  }

  public AutoRoutine l4CenterPreloadRightSource2() {
    final AutoRoutine routine = m_factory.newRoutine("l4CenterPreloadRightSource2");
    final AutoTrajectory preloadH = routine.trajectory("MID-H");
    final AutoTrajectory HtoSource = routine.trajectory("H-Source2");
    final AutoTrajectory SourceToC = routine.trajectory("Source2-C");
    final AutoTrajectory CToSource = routine.trajectory("C-Source2");
    final AutoTrajectory SourceToD = routine.trajectory("Source2-D");

    routine.active().onTrue(preloadH.resetOdometry().andThen(preloadH.cmd()));
    preloadH.atTimeBeforeEnd(.5).onTrue(m_autoCommands.goToL4());
    preloadH
        .done()
        .onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
                .andThen(m_autoCommands.scoreL4())
                .until(m_endEffector.coralBeamBreak.negate())
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () -> preloadH.getFinalPose().orElse(CoralTargets.BLUE_H.location)))
                .andThen(
                    m_autoCommands
                        .home()
                        .alongWith(Commands.waitSeconds(.5).andThen(HtoSource.spawnCmd()))));

    HtoSource.atTimeBeforeEnd(.5)
        .onTrue(
            m_autoCommands
                .goToSource()
                .until(m_endEffector.coralBeamBreak)
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () ->
                            HtoSource.getFinalPose()
                                .orElse(SourceIntakeTargets.SOURCE_R_BLUE.location)))
                .andThen(m_autoCommands.homeSource().alongWith(SourceToC.spawnCmd())));
    SourceToC.atTimeBeforeEnd(.5).onTrue(m_autoCommands.goToL4());
    SourceToC.done()
        .onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
                .andThen(m_autoCommands.scoreL4())
                .until(m_endEffector.coralBeamBreak.negate())
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () -> SourceToC.getFinalPose().orElse(CoralTargets.BLUE_C.location)))
                .andThen(
                    m_autoCommands
                        .home()
                        .alongWith(Commands.waitSeconds(.5).andThen(CToSource.spawnCmd()))));

    CToSource.atTimeBeforeEnd(.5)
        .onTrue(
            m_autoCommands
                .goToSource()
                .until(m_endEffector.coralBeamBreak)
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () ->
                            CToSource.getFinalPose()
                                .orElse(SourceIntakeTargets.SOURCE_R_BLUE.location)))
                .andThen(m_autoCommands.homeSource().alongWith(SourceToD.spawnCmd())));
    SourceToD.atTimeBeforeEnd(.5).onTrue(m_autoCommands.goToL4());
    SourceToD.done()
        .onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
                .andThen(m_autoCommands.scoreL4())
                .until(m_endEffector.coralBeamBreak.negate())
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () -> SourceToD.getFinalPose().orElse(CoralTargets.BLUE_D.location)))
                .andThen(m_autoCommands.home()));

    return routine;
  }

  public AutoRoutine l4RightPreloadRightSource2() {
    final AutoRoutine routine = m_factory.newRoutine("l4RightPreloadRightSource2");
    final AutoTrajectory preloadF = routine.trajectory("RIGHT-F");
    final AutoTrajectory FtoSource = routine.trajectory("F-Source2");
    final AutoTrajectory SourceToC = routine.trajectory("Source2-C");
    final AutoTrajectory CToSource = routine.trajectory("C-Source2");
    final AutoTrajectory SourceToD = routine.trajectory("Source2-D");

    routine.active().onTrue(preloadF.resetOdometry().andThen(preloadF.cmd()));
    preloadF.atTimeBeforeEnd(.5).onTrue(m_autoCommands.goToL4());
    preloadF
        .done()
        .onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
                .andThen(m_autoCommands.scoreL4())
                .until(m_endEffector.coralBeamBreak.negate())
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () -> preloadF.getFinalPose().orElse(CoralTargets.BLUE_H.location)))
                .andThen(
                    m_autoCommands
                        .home()
                        .alongWith(Commands.waitSeconds(.5).andThen(FtoSource.spawnCmd()))));

    FtoSource.atTimeBeforeEnd(.5)
        .onTrue(
            m_autoCommands
                .goToSource()
                .until(m_endEffector.coralBeamBreak)
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () ->
                            FtoSource.getFinalPose()
                                .orElse(SourceIntakeTargets.SOURCE_R_BLUE.location)))
                .andThen(m_autoCommands.homeSource().alongWith(SourceToC.spawnCmd())));
    SourceToC.atTimeBeforeEnd(.5).onTrue(m_autoCommands.goToL4());
    SourceToC.done()
        .onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
                .andThen(m_autoCommands.scoreL4())
                .until(m_endEffector.coralBeamBreak.negate())
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () -> SourceToC.getFinalPose().orElse(CoralTargets.BLUE_C.location)))
                .andThen(
                    m_autoCommands
                        .home()
                        .alongWith(Commands.waitSeconds(.5).andThen(CToSource.spawnCmd()))));

    CToSource.atTimeBeforeEnd(.5)
        .onTrue(
            m_autoCommands
                .goToSource()
                .until(m_endEffector.coralBeamBreak)
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () ->
                            CToSource.getFinalPose()
                                .orElse(SourceIntakeTargets.SOURCE_R_BLUE.location)))
                .andThen(m_autoCommands.homeSource().alongWith(SourceToD.spawnCmd())));
    SourceToD.atTimeBeforeEnd(.5).onTrue(m_autoCommands.goToL4());
    SourceToD.done()
        .onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
                .andThen(m_autoCommands.scoreL4())
                .until(m_endEffector.coralBeamBreak.negate())
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () -> SourceToD.getFinalPose().orElse(CoralTargets.BLUE_D.location)))
                .andThen(m_autoCommands.home()));

    return routine;
  }

  private static class AutoCommands {

    private final Elevator m_elevator;
    private final Arm m_arm;
    private final EndEffector m_endEffector;

    public AutoCommands(Elevator elevator, Arm arm, EndEffector endEffector) {

      m_elevator = elevator;
      m_arm = arm;
      m_endEffector = endEffector;
    }

    public Command goToL4() {
      return m_elevator.toReefLevel(3).alongWith(m_arm.toReefLevel(2, () -> true));
    }

    public Command scoreL4() {
      return m_endEffector.setL4Voltage(() -> true);
    }

    public Command goToSource() {
      return m_elevator
          .setPosition(ElevatorConstants.sourcePosition.in(Rotations))
          .alongWith(m_arm.toSourceLevel())
          .alongWith(m_endEffector.setSourceVelocity());
    }

    public Command home() {
      return m_elevator
          .toArmSafePosition()
          .alongWith(m_endEffector.coralOff())
          .until(m_elevator.isSafeForArm)
          .andThen(m_arm.toHome())
          .until(m_arm.isSafePosition)
          .andThen(m_elevator.toHome());
    }

    public Command homeSource() {
      return m_elevator
          .toArmSafePosition()
          .alongWith(m_endEffector.coralOff())
          .until(m_elevator.isSafeForArm)
          .andThen(m_arm.toHome(() -> false))
          .until(m_arm.isSafePosition)
          .andThen(m_elevator.toHome());
    }
  }
}
