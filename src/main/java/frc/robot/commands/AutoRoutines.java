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

public class AutoRoutines {
  private final AutoFactory m_factory;

  private final AutoCommands m_autoCommands;

  private final Elevator m_elevator;
  private final Arm m_arm;
  private final EndEffector m_endEffector;

  public AutoRoutines(AutoFactory factory, Elevator elevator, Arm arm, EndEffector endEffector) {
    m_factory = factory;
    m_elevator = elevator;
    m_arm = arm;
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

  public AutoRoutine l4Preload() {
    final AutoRoutine routine = m_factory.newRoutine("l4Preload");
    final AutoTrajectory l4Preload = routine.trajectory("MID-H");

    routine.active().onTrue(l4Preload.resetOdometry().andThen(l4Preload.cmd()));
    l4Preload.atTimeBeforeEnd(.5).onTrue(m_autoCommands.goToL4());
    l4Preload
        .done()
        .onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
                .andThen(m_autoCommands.scoreL4())
                .until(
                    m_endEffector.leftBeamBreak.negate().and(m_endEffector.rightBeamBreak.negate()))
                .andThen(m_autoCommands.home()));

    return routine;
  }

  public AutoRoutine l4PreloadBottomSource1() {
    final AutoRoutine routine = m_factory.newRoutine("l4PreloadBottomSource1");
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
                .until(
                    m_endEffector.leftBeamBreak.negate().and(m_endEffector.rightBeamBreak.negate()))
                .andThen(
                    m_autoCommands
                        .home()
                        .alongWith(Commands.waitSeconds(.5).andThen(HtoSource.spawnCmd()))));

    HtoSource.atTimeBeforeEnd(.5)
        .onTrue(
            m_autoCommands
                .goToSource()
                .until(m_endEffector.rightBeamBreak)
                .andThen(m_autoCommands.home().alongWith(SourceToC.spawnCmd())));
    SourceToC.atTimeBeforeEnd(.5).onTrue(m_autoCommands.goToL4());
    SourceToC.done()
        .onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
                .andThen(m_autoCommands.scoreL4())
                .until(
                    m_endEffector.leftBeamBreak.negate().and(m_endEffector.rightBeamBreak.negate()))
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
      return m_elevator.toReefLevel(3).alongWith(m_arm.toReefLevel(2, () -> false));
    }

    public Command scoreL4() {
      return m_endEffector.setL4Voltage(() -> false);
    }

    public Command goToSource() {
      return m_elevator
          .setPosition(ElevatorConstants.sourcePosition.in(Rotations))
          .alongWith(
              Commands.waitUntil(m_elevator.isSafeForArm).andThen(m_arm.toSourceLevel(() -> false)))
          .alongWith(m_endEffector.setSourceVelocity(() -> false));
    }

    public Command home() {
      return m_elevator
          .toArmSafePosition()
          .alongWith(Commands.waitUntil(m_elevator.isSafeForArm).andThen(m_arm.toHome(() -> false)))
          .alongWith(m_endEffector.coralOff())
          .alongWith(Commands.waitUntil(m_arm.isSafePosition).andThen(m_elevator.toHome()));
    }
  }
}
