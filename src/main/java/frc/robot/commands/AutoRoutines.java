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
import frc.robot.subsystems.rollers.Roller;

public class AutoRoutines {
  private final AutoFactory m_factory;

  private final Roller roller;

  public AutoRoutines(AutoFactory factory, Roller roller) {
    m_factory = factory;
    this.roller = roller;
  }

  public AutoRoutine simplePathAuto() {
    final AutoRoutine routine = m_factory.newRoutine("test");
    final AutoTrajectory simplePath = routine.trajectory("test");

    routine.active().onTrue(simplePath.resetOdometry().andThen(simplePath.cmd()));
    return routine;
  }

  private static class AutoCommands {
    private AutoCommands() {
      throw new UnsupportedOperationException("This is a utility class!");
    }
  }
}
