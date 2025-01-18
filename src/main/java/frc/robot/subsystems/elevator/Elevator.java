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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.utils.DisableSubsystem;
import org.littletonrobotics.junction.Logger;

public class Elevator extends DisableSubsystem {

  private final ElevatorIO motorIO;
  private final ElevatorIOInputsAutoLogged motorIOAutoLogged = new ElevatorIOInputsAutoLogged();
  private final SysIdRoutine m_sysIdRoutine;

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

  public Command setPosition(Angle position) {
    return this.run(
        () ->
            motorIO.setPosition(
                position.in(Rotations) * ElevatorConstants.SimulationConstants.kGearRatio));
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
      return this.setPosition(ElevatorConstants.kReefPositions[level]);
    } else {
      return this.setPosition(
          Rotations.of(
              ElevatorConstants.SimulationConstants.kReefPositions[level]
                      .div(ElevatorConstants.SimulationConstants.kWheelRadius)
                      .magnitude()
                  / (2 * Math.PI)));
    }
  }

  public Command toSource() {
    return this.setPosition(ElevatorConstants.kSourcePosition);
  }

  public Command toGround() {
    return this.setPosition(ElevatorConstants.kGroundPosition);
  }

  // TODO: Grab coral level from NT/selector

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
