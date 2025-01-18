// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.DisableSubsystem;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends DisableSubsystem {

  private final EndEffectorIO endEffectorIO;
  private final EndEffectorIOInputsAutoLogged endEffectorIOInputsAutoLogged =
      new EndEffectorIOInputsAutoLogged();

  public EndEffector(boolean enabled, EndEffectorIO endEffectorIO) {
    super(enabled);
    this.endEffectorIO = endEffectorIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    endEffectorIO.updateInputs(endEffectorIOInputsAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), endEffectorIOInputsAutoLogged);
  }

  public Command setVoltage(double algaeVoltage, double coralVoltage) {
    return this.run(
            () -> {
              endEffectorIO.setAlgaeVoltage(algaeVoltage);
              endEffectorIO.setCoralVoltage(coralVoltage);
            })
        .finallyDo(endEffectorIO::off);
  }

  public Command setVelocity(AngularVelocity algaeVelocity, AngularVelocity coralVelocity) {
    return this.run(
            () -> {
              endEffectorIO.setAlgaeVelocity(algaeVelocity);
              endEffectorIO.setCoralVelocity(coralVelocity);
            })
        .finallyDo(endEffectorIO::off);
  }

  public Command off() {
    return this.runOnce(endEffectorIO::off);
  }
}
