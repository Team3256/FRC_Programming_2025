// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.CoralGroundIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.DisableSubsystem;
import org.littletonrobotics.junction.Logger;

public class CoralGroundIntake extends DisableSubsystem {
  private final CoralIntakeIO intakeIO;
  private final CoralIntakeIOInputsAutoLogged coralIOAutoLogged =
      new CoralIntakeIOInputsAutoLogged();

  private final Trigger debouncedBeamBreak = new Trigger(this::isBeamBroken).debounce(0.1);
  ;

  public CoralGroundIntake(boolean disabled, CoralIntakeIO intakeIO) {
    super(disabled);

    this.intakeIO = intakeIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    intakeIO.updateInputs(coralIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), coralIOAutoLogged);
  }

  public Command setVoltage(double voltage, double passthroughVoltage) {
    return this.run(
            () -> {
              intakeIO.setIntakeVoltage(voltage);
              intakeIO.setLinearMotorVoltage(passthroughVoltage);
            })
        .finallyDo(intakeIO::off);
  }

  public Command setVelocity(double velocity, double passthroughVelocity) {
    return this.run(
            () -> {
              intakeIO.setIntakeVelocity(velocity);
              intakeIO.setLinearMotorVelocity(passthroughVelocity);
            })
        .finallyDo(intakeIO::off);
  }

  public Command setIntakeVoltage(double voltage) {
    return this.run(() -> intakeIO.setIntakeVoltage(voltage)).finallyDo(intakeIO::off);
  }

  public Command setIntakeVelocity(double velocity) {
    return this.run(() -> intakeIO.setIntakeVelocity(velocity)).finallyDo(intakeIO::off);
  }

  public Command setLinearMotorVoltage(double voltage) {
    return this.run(() -> intakeIO.setLinearMotorVoltage(voltage)).finallyDo(intakeIO::off);
  }

  public Command setLinearMotorVelocity(double velocity) {
    return this.run(() -> intakeIO.setLinearMotorVelocity(velocity)).finallyDo(intakeIO::off);
  }

  public Command off() {
    return this.runOnce(intakeIO::off);
  }

  public Command setIntakeVelocityPassthroughVoltage(double velocity, double voltage) {
    return setIntakeVelocity(velocity).andThen(setLinearMotorVoltage(voltage));
  }

  public Command intakeIn() {
    return this.run(
            () -> {
              intakeIO.setIntakeVelocity(80);
              intakeIO.setLinearMotorVoltage(CoralGroundIntakeConstants.kLinearSlideMotorVoltage);
            })
        .until(debouncedBeamBreak)
        .andThen(this.off());
  }

  public boolean isBeamBroken() {
    return intakeIO.isBeamBroken();
  }
}
