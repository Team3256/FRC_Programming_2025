// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.autogen;

import static edu.wpi.first.units.Units.Seconds;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class TrajTriggers {
  private TrajTriggers() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Trigger atTimeToEnd(AutoTrajectory traj, Time time) {
    return atTimeToEnd(traj, time.in(Seconds));
  }

  public static Trigger atTimeToEnd(AutoTrajectory traj, double time) {
    return traj.atTime(traj.getRawTrajectory().getTotalTime() - time);
  }
}
