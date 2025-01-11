// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class RollerIOTalonFX implements RollerIO {

  private final TalonFX motor;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VelocityTorqueCurrentFOC focRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);
  private final VoltageOut voltageReq = new VoltageOut(0);

  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<Current> motorStatorCurrent;
  private final StatusSignal<Current> motorSupplyCurrent;
  // private final StatusSignal<Double> motorTemperature;
  private final StatusSignal<Double> motorReferenceSlope;

  public RollerIOTalonFX() {
    motor = new TalonFX(EndeffectorConstants.kRollerMotorID);
    motorVoltage = motor.getMotorVoltage();
    motorPosition = motor.getPosition();
    motorStatorCurrent = motor.getStatorCurrent();
    motorSupplyCurrent = motor.getSupplyCurrent();
    // motorTemperature = motor.getDeviceTemp();
    motorReferenceSlope = motor.getClosedLoopReferenceSlope();

    PhoenixUtil.applyMotorConfigs(
        motor, EndeffectorConstants.kRollerMotorConfig, EndeffectorConstants.kFlashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        EndeffectorConstants.kStatusSignalUpdateFrequency,
        motorVoltage,
        motorPosition,
        motorStatorCurrent,
        motorSupplyCurrent,
        // motorTemperature,
        motorReferenceSlope);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        motorVoltage,
        motorPosition,
        motorStatorCurrent,
        motorSupplyCurrent,
        // motorTemperature,
        motorReferenceSlope);
    inputs.motorVoltage = motorVoltage.getValueAsDouble();
    inputs.motorPosition = motorPosition.getValueAsDouble();
    inputs.motorStatorCurrent = motorStatorCurrent.getValueAsDouble();
    inputs.motorSupplyCurrent = motorSupplyCurrent.getValueAsDouble();
    // inputs.motorTemperature = motorTemperature.getValueAsDouble();
    inputs.motorReferenceSlope = motorReferenceSlope.getValueAsDouble();
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    if (EndeffectorConstants.kUseFOC) {
      motor.setControl(focRequest.withVelocity(velocity));
    } else {
      motor.setControl(velocityRequest.withVelocity(velocity));
    }
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void off() {
    motor.setControl(new NeutralOut());
  }

  @Override
  public void zero() {
    motor.setPosition(0);
  }

  @Override
  public TalonFX getMotor() {
    return motor;
  }

  @Override
  public VoltageOut getVoltageRequest() {
    return voltageReq;
  }
}
