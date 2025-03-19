// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends DisableSubsystem {

  private final EndEffectorIO endEffectorIO;
  private final EndEffectorIOInputsAutoLogged endEffectorIOInputsAutoLogged =
      new EndEffectorIOInputsAutoLogged();

  public final Trigger rightBeamBreak =
      new Trigger(() -> endEffectorIOInputsAutoLogged.rightBeamBreak);
  public final Trigger leftBeamBreak =
      new Trigger(() -> endEffectorIOInputsAutoLogged.leftBeamBreak);

  public EndEffector(boolean enabled, EndEffectorIO endEffectorIO) {
    super(enabled);
    this.endEffectorIO = endEffectorIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    endEffectorIO.updateInputs(endEffectorIOInputsAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), endEffectorIOInputsAutoLogged);

    LoggedTracer.record(this.getClass().getSimpleName());
  }

  public Command setCoralVoltage(DoubleSupplier voltage) {
    return this.run(() -> endEffectorIO.setCoralVoltage(voltage.getAsDouble()));
  }

  public Command setCoralVelocity(Supplier<AngularVelocity> velocity) {
    return this.run(() -> endEffectorIO.setCoralVelocity(velocity.get()));
  }

  public Command setAlgaeVoltage(double voltage) {
    return this.run(() -> endEffectorIO.setAlgaeVoltage(voltage));
  }

  public Command setAlgaeVelocity(Supplier<AngularVelocity> velocity) {
    return this.run(() -> endEffectorIO.setAlgaeVelocity(velocity.get()));
  }

  public Command setL1Velocity(BooleanSupplier rightSide) {
    return setCoralVelocity(
        () ->
            rightSide.getAsBoolean()
                ? EndEffectorConstants.l1Velocity
                : EndEffectorConstants.l1Velocity.times(-1));
  }

  public Command setL2L3Velocity(BooleanSupplier rightSide) {
    return setCoralVelocity(
        () ->
            rightSide.getAsBoolean()
                ? EndEffectorConstants.l2l3Velocity
                : EndEffectorConstants.l2l3Velocity.times(-1));
  }

  public Command setL4Voltage(BooleanSupplier rightSide) {
    return setCoralVoltage(
        () ->
            rightSide.getAsBoolean()
                ? EndEffectorConstants.l4Voltage
                : EndEffectorConstants.l4Voltage * -1);
  }

  public Command setSourceVelocity(BooleanSupplier rightSide) {
    return setCoralVelocity(
        () ->
            rightSide.getAsBoolean()
                ? EndEffectorConstants.sourceVelocity
                : EndEffectorConstants.sourceVelocity.times(-1));
  }

  public Command setAlgaeIntakeVelocity() {
    return setAlgaeVelocity(() -> EndEffectorConstants.algaeIntakeVelocity);
  }

  public Command setAlgaeOuttakeVoltage() {
    return setAlgaeVelocity(() -> EndEffectorConstants.algaeOuttakeVelocity);
  }

  public Command algaeOff() {
    return this.runOnce(endEffectorIO::algaeOff);
  }

  public Command coralOff() {
    return this.runOnce(endEffectorIO::coralOff);
  }
}
