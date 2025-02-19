// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.DisableSubsystem;
import org.littletonrobotics.junction.Logger;

public class Climb extends DisableSubsystem {
  private final ClimbIO climbIO;
  private final ClimbIOInputsAutoLogged climbIOAutoLogged = new ClimbIOInputsAutoLogged();

  public Climb(boolean enabled, ClimbIO climbIO) {
    super(enabled);
    this.climbIO = climbIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    climbIO.updateInputs(climbIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), climbIOAutoLogged);
  }

  public Command setPosition(double position) {
    return this.run(() -> climbIO.setPosition(position));
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> climbIO.setVoltage(voltage));
  }

  public Command off() {
    return this.runOnce(climbIO::off);
  }

  public Command zero() {
    return this.runOnce(climbIO::zero);
  }
}
