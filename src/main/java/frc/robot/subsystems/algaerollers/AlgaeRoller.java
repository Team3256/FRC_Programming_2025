// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.algaerollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import org.littletonrobotics.junction.Logger;

public class AlgaeRoller extends DisableSubsystem {

  private final AlgaeRollerIO algaeRollerIO;
  private final AlgaeRollerIOInputsAutoLogged algaeRollerIOInputsAutoLogged =
      new AlgaeRollerIOInputsAutoLogged();

  public AlgaeRoller(boolean enabled, AlgaeRollerIO endEffectorIO) {
    super(enabled);
    this.algaeRollerIO = endEffectorIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    algaeRollerIO.updateInputs(algaeRollerIOInputsAutoLogged);
    Logger.processInputs("AlgaeRoller", algaeRollerIOInputsAutoLogged);

    LoggedTracer.record("AlgaeRoller");
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> algaeRollerIO.setAlgaeVoltage(voltage));
  }

  public Command setIntakeVoltage() {
    return setVoltage(AlgaeRollerConstants.intakeVoltage).withName("setIntakeVoltage");
  }

  public Command setProcessorVoltage() {
    return setVoltage(AlgaeRollerConstants.processorVoltage).withName("setProcessorVoltage");
  }

  public Command setL1Voltage() {
    return setVoltage(AlgaeRollerConstants.l1Voltage).withName("setL1Voltage");
  }

  public Command off() {
    return this.runOnce(algaeRollerIO::algaeOff).withName("off");
  }
}
