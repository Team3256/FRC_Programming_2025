// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.algaearm;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import frc.robot.utils.Util;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AlgaeArm extends DisableSubsystem {

  private final AlgaeArmIO algaeArmIO;
  private final AlgaeArmIOInputsAutoLogged algaeArmIOInputsAutoLogged =
      new AlgaeArmIOInputsAutoLogged();

  public final Trigger reachedPosition = new Trigger(this::isAtPosition);

  private final MutAngle requestedPosition = Rotations.of(0.0).mutableCopy();

  public AlgaeArm(boolean enabled, AlgaeArmIO armIO) {
    super(enabled);

    this.algaeArmIO = armIO;
    algaeArmIO.resetPosition(Rotations.of(.25));
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.recordOutput("AlgaeArm" + "/requestedPosition", requestedPosition.in(Rotations));
    algaeArmIO.updateInputs(algaeArmIOInputsAutoLogged);
    Logger.processInputs("AlgaeArm", algaeArmIOInputsAutoLogged);

    LoggedTracer.record("AlgaeArm");
  }

  public Command setPosition(Supplier<Angle> position) {
    return this.run(
        () -> {
          requestedPosition.mut_replace(position.get());
          algaeArmIO.setPosition(requestedPosition);
        });
  }

  public Command setPosition(Angle position) {
    return setPosition(() -> position);
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> algaeArmIO.setVoltage(voltage));
  }

  @AutoLogOutput
  public boolean isAtPosition() {
    return Util.epsilonEquals(
        algaeArmIOInputsAutoLogged.algaeArmMotorPosition, requestedPosition.in(Rotations), 0.02);
  }

  public Command toHome() {
    return this.setPosition(AlgaeArmConstants.homePosition).withName("toHome");
  }

  public Command toGroundAlgae() {
    return this.setPosition(AlgaeArmConstants.groundAlgaePosition).withName("toGroundAlgae");
  }

  public Command toGroundAlgaeCoast() {
    return this.setPosition(AlgaeArmConstants.groundAlgaePosition)
        .until(reachedPosition.debounce(.03))
        .andThen(this.off());
  }

  public Command toPartialDeploy() {
    return this.setPosition(AlgaeArmConstants.partialDeploy)
        .until(reachedPosition.debounce(.03))
        .andThen(this.off())
        .withName("toPartialDeploy");
  }

  public Command toL1() {
    return this.setPosition(AlgaeArmConstants.l1Position).withName("toL1");
  }

  public Command off() {
    return this.runOnce(algaeArmIO::off);
  }
}
