// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
  private final AutoFactory m_factory;

  public AutoRoutines(AutoFactory factory) {
    m_factory = factory;
  }

  /*
   * Demo auto
   */
  public AutoRoutine simplePathAuto() {
    final AutoRoutine routine = m_factory.newRoutine("Source2 - A");
    final AutoTrajectory simplePath = routine.trajectory("Source2-A");

    routine.active().onTrue(simplePath.resetOdometry().andThen(simplePath.cmd()));
    return routine;
  }

  public AutoRoutine ds1MobilityAuto() {
    final AutoRoutine routine = m_factory.newRoutine("Mobility Auto");
    final AutoTrajectory path = routine.trajectory("DS1-Mobility");

    routine.active().onTrue(path.resetOdometry().andThen(path.cmd()));

    return routine;
  }

  public AutoRoutine ds2MobilityAuto() {
    final AutoRoutine routine = m_factory.newRoutine("Mobility Auto");
    final AutoTrajectory path = routine.trajectory("DS2-Mobility");

    routine.active().onTrue(path.resetOdometry().andThen(path.cmd()));

    return routine;
  }

  public AutoRoutine ds3MobilityAuto() {
    final AutoRoutine routine = m_factory.newRoutine("Mobility Auto");
    final AutoTrajectory path = routine.trajectory("DS3-Mobility");

    routine.active().onTrue(path.resetOdometry().andThen(path.cmd()));

    return routine;
  }

  private static class AutoCommands {
    private AutoCommands() {
      throw new UnsupportedOperationException("This is a utility class!");
    }
  }
}
