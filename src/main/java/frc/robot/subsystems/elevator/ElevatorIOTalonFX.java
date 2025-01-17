// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class ElevatorIOTalonFX implements ElevatorIO {

  private final TalonFX motor;
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
  private final VoltageOut voltageReq = new VoltageOut(0);

  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<Current> motorStatorCurrent;
  private final StatusSignal<Current> motorSupplyCurrent;
  // private final StatusSignal<Double> motorTemperature;
  private final StatusSignal<Double> motorReferenceSlope;

  public ElevatorIOTalonFX() {
    motor = new TalonFX(ElevatorConstants.kMotorID);
    motorVoltage = motor.getMotorVoltage();
    motorPosition = motor.getPosition();
    motorStatorCurrent = motor.getStatorCurrent();
    motorSupplyCurrent = motor.getSupplyCurrent();
    // motorTemperature = motor.getDeviceTemp();
    motorReferenceSlope = motor.getClosedLoopReferenceSlope();

    PhoenixUtil.applyMotorConfigs(
        motor, ElevatorConstants.kMotorConfig, ElevatorConstants.kFlashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        ElevatorConstants.kStatusSignalUpdateFrequency,
        motorVoltage,
        motorPosition,
        motorStatorCurrent,
        motorSupplyCurrent,
        // motorTemperature,
        motorReferenceSlope);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
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
  public void setPosition(double position) {
    if (ElevatorConstants.kUseMotionMagic) {
      motor.setControl(motionMagicRequest.withPosition(position));
    } else {
      motor.setControl(positionRequest.withPosition(position));
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
