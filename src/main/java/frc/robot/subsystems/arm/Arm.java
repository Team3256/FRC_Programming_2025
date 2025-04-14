// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.LoggedTracer;
import frc.robot.utils.Util;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends DisableSubsystem {

  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged armIOAutoLogged = new ArmIOInputsAutoLogged();

  public final Trigger reachedPosition = new Trigger(this::isAtPosition);
  public final Trigger isSafePosition = new Trigger(this::isSafePosition);

  private double cachedArmMotorPosition = 0.0;
  private int cachedDirection = 0;
  private final MutAngle requestedPosition = Rotations.of(0.0).mutableCopy();

  public Arm(boolean enabled, ArmIO armIO) {
    super(enabled);

    this.armIO = armIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.recordOutput(
        this.getClass().getSimpleName() + "/requestedPosition", requestedPosition.in(Rotations));
    armIO.updateInputs(armIOAutoLogged);
    Logger.processInputs("Arm", armIOAutoLogged);

    LoggedTracer.record("Arm");
  }

  public Command setPosition(Supplier<Angle> position, boolean continuous, IntSupplier direction) {
    return this.run(
            () -> {
              cachedArmMotorPosition =
                  direction.getAsInt() == cachedDirection
                      ? cachedArmMotorPosition
                      : armIOAutoLogged.armMotorPosition;
              requestedPosition.mut_replace(
                  continuous
                      ? continuousWrapAtHome(position.get(), direction.getAsInt())
                      : position.get());
              armIO.setPosition(requestedPosition);
              cachedDirection = direction.getAsInt();
            })
        .beforeStarting(() -> cachedArmMotorPosition = armIOAutoLogged.armMotorPosition);
  }

  public Command setPosition(Angle position, boolean continuous, int direction) {
    return setPosition(() -> position, continuous, () -> direction);
  }

  public Command setPosition(double position, boolean continuous, int direction) {
    return setPosition(() -> Rotations.of(position), continuous, () -> direction);
  }

  public Command setPosition(DoubleSupplier position, boolean continuous, IntSupplier direction) {
    return setPosition(() -> Rotations.of(position.getAsDouble()), continuous, direction);
  }

  public Command setVoltage(Voltage voltage) {
    return this.run(() -> armIO.setVoltage(voltage));
  }

  public Command toReefLevel(int level, BooleanSupplier rightSide) {
    return setPosition(
            () ->
                rightSide.getAsBoolean()
                    ? ArmConstants.reefRightPositions[level]
                    : ArmConstants.reefLeftPositions[level],
            true,
            () -> 0)
        .withName("toReefLevel_" + level);
  }

  public Command toDealgaeLevel(int level, BooleanSupplier rightSide) {
    return this.setPosition(
            () ->
                rightSide.getAsBoolean()
                    ? ArmConstants.dealgaeRightPosition[level]
                    : ArmConstants.dealgaeLeftPosition[level],
            true,
            () -> 0)
        .withName("toDealgaeLevel_" + level);
  }

  public Command toClimb() {
    return this.setPosition(() -> ArmConstants.climbPosition, true, () -> 0).withName("toClimb");
  }

  public Command toProcessorLevel() {
    return this.setPosition(() -> ArmConstants.processorRightPosition, true, () -> 0)
        .withName("toProcessorLevel");
  }

  @AutoLogOutput
  public boolean isSafePosition() {
    return (armIOAutoLogged.armMotorPosition + 5) % 1 >= ArmConstants.safeLeftPosition
        && (armIOAutoLogged.armMotorPosition + 5) % 1 <= ArmConstants.safeRightPosition;
  }

  public Command toSourceLevel() {
    return this.setPosition(() -> ArmConstants.sourcePosition, true, () -> 0)
        .withName("toSourceLevel");
  }

  public Command toBargeLevel(BooleanSupplier rightSide) {
    return this.setPosition(
            () ->
                rightSide.getAsBoolean()
                    ? ArmConstants.bargeRightPosition
                    : ArmConstants.bargeLeftPosition,
            true,
            () -> 0)
        .withName("toBargeLevel");
  }

  public Command toGroundAlgaeLevel() {
    return this.setPosition(() -> ArmConstants.groundAlgaeRightPosition, true, () -> 0)
        .withName("toGroundAlgaeLevel");
  }

  @AutoLogOutput
  public boolean isAtPosition() {
    return Util.epsilonEquals(
        armIOAutoLogged.armMotorPosition, requestedPosition.in(Rotations), 0.05);
  }

  public Command toHome() {
    return this.setPosition(ArmConstants.homePosition, true, 0).withName("toHome");
  }

  public Command off() {
    return this.runOnce(armIO::off).withName("off");
  }

  public Angle continuousWrapAtHome(Angle angle, int direction) {
    return Rotations.of(continuousWrapAtHome(angle.in(Rotations), direction));
  }

  public double continuousWrapAtHome(double angle, int direction) {
    return continuousWrapAtHome(angle, cachedArmMotorPosition, direction);
  }

  public static double continuousWrapAtHome(
      double reqAbsAngle, double currentAngle, double forcedDirection) {
    int n_min = (int) Math.ceil(-ArmConstants.maxRotations.in(Rotations) - reqAbsAngle);
    int n_max = (int) Math.floor(ArmConstants.maxRotations.in(Rotations) - reqAbsAngle);
    int nIdeal = (int) Math.round(currentAngle - reqAbsAngle);
    int nCandidate = Math.min(n_max, Math.max(n_min, nIdeal));
    double candidate = reqAbsAngle + nCandidate;
    double diff = candidate - currentAngle;

    int adjustment =
        (int)
            ((1 - Math.max(Math.signum(diff) * Math.signum(forcedDirection), 0))
                * Math.signum(forcedDirection));

    int nLong = nCandidate + adjustment;
    nLong = Math.min(n_max, Math.max(n_min, nLong));
    return reqAbsAngle + nLong;
  }
}
