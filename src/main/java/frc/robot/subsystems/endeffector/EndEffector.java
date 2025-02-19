// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.DisableSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends DisableSubsystem {

  private final EndEffectorIO endEffectorIO;
  private final EndEffectorIOInputsAutoLogged endEffectorIOInputsAutoLogged =
      new EndEffectorIOInputsAutoLogged();

  public final Trigger beamBreak = new Trigger(() -> endEffectorIOInputsAutoLogged.beamBreak);

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

  public Command setVelocity(
      Supplier<AngularVelocity> algaeVelocity, Supplier<AngularVelocity> coralVelocity) {
    return this.run(
            () -> {
              endEffectorIO.setAlgaeVelocity(algaeVelocity.get());
              endEffectorIO.setCoralVelocity(coralVelocity.get());
            })
        .finallyDo(endEffectorIO::off);
  }

  public Command setVelocity(AngularVelocity algaeVelocity, AngularVelocity coralVelocity) {
    return setVelocity(() -> algaeVelocity, () -> coralVelocity);
  }

  public Command setL1Velocity(BooleanSupplier rightSide) {
    return setVelocity(
        () -> RotationsPerSecond.of(1),
        () ->
            rightSide.getAsBoolean()
                ? EndEffectorConstants.l1Velocity.times(-1)
                : EndEffectorConstants.l1Velocity);
  }

  public Command setL2L3Velocity(BooleanSupplier rightSide) {
    return setVelocity(
        () -> RotationsPerSecond.of(1),
        () ->
            rightSide.getAsBoolean()
                ? EndEffectorConstants.l2l3Velocity.times(-1)
                : EndEffectorConstants.l2l3Velocity);
  }

  public Command setL4Velocity(BooleanSupplier rightSide) {
    return setVelocity(
        () -> RotationsPerSecond.of(1),
        () ->
            rightSide.getAsBoolean()
                ? EndEffectorConstants.l4Velocity.times(-1)
                : EndEffectorConstants.l4Velocity);
  }

  public Command setSourceVelocity(BooleanSupplier rightSide) {
    return setVelocity(
        () -> EndEffectorConstants.sourceVelocity[0],
        () ->
            rightSide.getAsBoolean()
                ? EndEffectorConstants.sourceVelocity[1].times(-1)
                : EndEffectorConstants.sourceVelocity[1]);
  }

  public Command off() {
    return this.runOnce(endEffectorIO::off);
  }
}
