// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public abstract class DebugCommand extends Command {

  @Override
  public void initialize() {
    if (Constants.FeatureFlags.DebugCommandEnabled) {
      // This should ideally be used in conjunction with AdvantageKit:
      // https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/DATA-FLOW.md#deterministic-timestamps
      // AdvKit injects a shim into Timer.getFPGATimestamp that allows us to get a
      // deterministic
      // timestamp.
      // This does mean that all commands in the same periodic() will have the same
      // timestamp, but I
      // don't believe that should be an issue for now.
      //      Logger.recordOutput("dcb/initialized", );

      System.out.println(
          "[DCB] "
              + this.getClass().getSimpleName()
              + " initialized @ "
              + edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (Constants.FeatureFlags.DebugCommandEnabled) {
      System.out.println(
          "[DCB] "
              + this.getClass().getSimpleName()
              + " ended (interrupted: "
              + interrupted
              + ") @ "
              + edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
    }
  }
}
