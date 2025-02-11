// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.Util;
import org.littletonrobotics.junction.Logger;

public class Elevator extends DisableSubsystem {

  private final ElevatorIO motorIO;
  private final ElevatorIOInputsAutoLogged motorIOAutoLogged = new ElevatorIOInputsAutoLogged();
  private final SysIdRoutine m_sysIdRoutine;

  private double requestedPosition = 0;

  public final Trigger reachedPosition = new Trigger(this::isAtPosition);

  public Elevator(boolean enabled, ElevatorIO motorIO) {
    super(enabled);
    this.motorIO = motorIO;
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Seconds), // Use default ramp rate (1 V/s)
                Volts.of(6), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) ->
                    motorIO
                        .getMotor()
                        .setControl(motorIO.getVoltageRequest().withOutput(volts.in(Volts))),
                null,
                this));
  }

  @Override
  public void periodic() {
    super.periodic();
    motorIO.updateInputs(motorIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), motorIOAutoLogged);
  }

  public Command setPosition(double position) {
    return this.run(
        () -> {
          requestedPosition = position;
          motorIO.setPosition(position);
        });
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> motorIO.setVoltage(voltage));
  }

  public Command off() {
    return this.runOnce(motorIO::off);
  }

  public Command zero() {
    return this.runOnce(motorIO::zero);
  }

  // Level must be between 0 to 3
  public Command toReefLevel(int level) {
    if (Robot.isReal()) {
      return this.setPosition(ElevatorConstants.kReefPositions[level].in(Rotations));
    } else {
      return this.setPosition(
          ElevatorConstants.SimulationConstants.kReefPositions[level]
                  .div(ElevatorConstants.SimulationConstants.kDrumRadius)
                  .magnitude()
              / (2 * Math.PI));
    }
  }

  public Command toDealgaeLevel(int level) {
    if (Robot.isReal()) {
      return this.setPosition(ElevatorConstants.kDealgaePositions[level].in(Rotations));
    } else {
      return this.setPosition(
          ElevatorConstants.SimulationConstants.kReefPositions[level]
                  .div(ElevatorConstants.SimulationConstants.kDrumRadius)
                  .magnitude()
              / (2 * Math.PI)
              * ElevatorConstants.SimulationConstants.kGearRatio);
    }
  }

  public Angle getModulusPosition() {
    return Rotations.of(
        MathUtil.inputModulus(
                (motorIOAutoLogged.encoderAAbsolutePosition
                    - motorIOAutoLogged.encoderBAbsolutePosition),
                0,
                1)
            * ((double) ElevatorConstants.kEncoderBTeethCount
                / (ElevatorConstants.kEncoderBTeethCount - ElevatorConstants.kEncoderATeethCount)));
  }

  public Command toArmSafePosition() {
    return this.setPosition(ElevatorConstants.armSafePosition.in(Rotations));
  }

  public boolean isAtPosition() {
    return Util.epsilonEquals(motorIOAutoLogged.motorPosition, requestedPosition, 0.05);
  }

  public Command toHome() {
    return this.setPosition(ElevatorConstants.homePosition.in(Rotations));
  }

  // TODO: Grab coral level from NT/selector

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
