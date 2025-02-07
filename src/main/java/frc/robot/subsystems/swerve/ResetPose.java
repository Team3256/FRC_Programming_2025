// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ResetPose extends InstantCommand {
  private CommandSwerveDrivetrain swerveSubsystem;

  public ResetPose(CommandSwerveDrivetrain swerve) {
    this.swerveSubsystem = swerve;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void execute() {
    this.swerveSubsystem.resetPose(swerveSubsystem.getCurrentPose());
  }
}
