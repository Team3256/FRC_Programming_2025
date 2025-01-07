// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils.generics;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class VelocityIOTalonFX<T extends SingleMotorConstants> implements VelocityIO {

  private final TalonFX motor;
  private final VoltageOut voltageReq = new VoltageOut(0);

  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Current> motorStatorCurrent;
  private final StatusSignal<Current> motorSupplyCurrent;
  private final StatusSignal<Temperature> motorTemperature;
  private final StatusSignal<Double> motorReferenceSlope;

  public VelocityIOTalonFX() {
    motor = new TalonFX(T.kMotorID);
    motorVoltage = motor.getMotorVoltage();
    motorVelocity = motor.getVelocity();
    motorStatorCurrent = motor.getStatorCurrent();
    motorSupplyCurrent = motor.getSupplyCurrent();
    motorTemperature = motor.getDeviceTemp();
    motorReferenceSlope = motor.getClosedLoopReferenceSlope();

    PhoenixUtil.applyMotorConfigs(motor, T.kMotorConfig, T.kFlashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        T.kStatusSignalUpdateFrequency,
        motorVoltage,
        motorVelocity,
        motorStatorCurrent,
        motorSupplyCurrent,
        motorTemperature,
        motorReferenceSlope);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(SingleMotorSubsystemInputs inputs) {
    BaseStatusSignal.refreshAll(
        motorVoltage,
        motorVelocity,
        motorStatorCurrent,
        motorSupplyCurrent,
        motorTemperature,
        motorReferenceSlope);
    inputs.motorVoltage = motorVoltage.getValueAsDouble();
    inputs.motorVelocity = motorVelocity.getValueAsDouble();
    inputs.motorStatorCurrent = motorStatorCurrent.getValueAsDouble();
    inputs.motorSupplyCurrent = motorSupplyCurrent.getValueAsDouble();
    inputs.motorTemperature = motorTemperature.getValueAsDouble();
    inputs.motorReferenceSlope = motorReferenceSlope.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void off() {
    // Or VelocityTorqueCurrentFOC for regen braking
    motor.setControl(new NeutralOut());
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
