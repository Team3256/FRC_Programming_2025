// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.DisableSubsystem;
import org.littletonrobotics.junction.Logger;

public class Roller extends DisableSubsystem {

  private final RollerIO rollerIO;

  private final RollerIOInputsAutoLogged rollerIOInputsAutoLogged = new RollerIOInputsAutoLogged();

  public Roller(boolean enabled, RollerIO rollerIO) {
    super(enabled);
    this.rollerIO = rollerIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
    rollerIO.updateInputs(rollerIOInputsAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), rollerIOInputsAutoLogged);
  }

  public Command off() {
    return this.runOnce(rollerIO::off);
  }

  public Command setRollerVoltage(double voltage) {
    return this.run(() -> rollerIO.setVoltage(voltage)).finallyDo(rollerIO::off);
  }
}
