// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.CoralGroundIntake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.utils.PhoenixUtil;

public class CoralIntakeIOTalonFX implements CoralIntakeIO {
  private final TalonFX intakeMotor = new TalonFX(CoralGroundIntakeConstants.kCGIMotorID);
  final VelocityVoltage intakeRequest = new VelocityVoltage(0).withSlot(0);
  final MotionMagicVelocityVoltage motionMagicIntakeRequest =
      new MotionMagicVelocityVoltage(0).withSlot(0);
  private final VoltageOut intakeVoltageReq = new VoltageOut(0);

  private final StatusSignal<Voltage> intakeMotorVoltage = intakeMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> intakeMotorVelocity = intakeMotor.getVelocity();
  private final StatusSignal<Current> intakeMotorStatorCurrent = intakeMotor.getStatorCurrent();
  private final StatusSignal<Current> intakeMotorSupplyCurrent = intakeMotor.getSupplyCurrent();
  private final StatusSignal<Temperature> intakeMotorTemperature = intakeMotor.getDeviceTemp();
  private final StatusSignal<Double> intakeMotorReferenceSlope =
      intakeMotor.getClosedLoopReferenceSlope();

  private final TalonFX linearSlideMotor =
      new TalonFX(CoralGroundIntakeConstants.kLinearSlideMotorID);
  final VelocityVoltage linearSlideRequest = new VelocityVoltage(0).withSlot(0);
  final MotionMagicVelocityVoltage motionMagicLinearSlideRequest =
      new MotionMagicVelocityVoltage(0).withSlot(0);
  private final VoltageOut linearSlideReq = new VoltageOut(0);

  private final StatusSignal<Voltage> passthroughMotorVoltage = linearSlideMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> passthroughMotorVelocity =
      linearSlideMotor.getVelocity();
  private final StatusSignal<Current> passthroughMotorStatorCurrent =
      linearSlideMotor.getStatorCurrent();
  private final StatusSignal<Current> passthroughMotorSupplyCurrent =
      linearSlideMotor.getSupplyCurrent();
  private final StatusSignal<Temperature> passthroughMotorTemperature =
      linearSlideMotor.getDeviceTemp();
  private final StatusSignal<Double> passthroughMotorReferenceSlope =
      linearSlideMotor.getClosedLoopReferenceSlope();

  private DigitalInput beamBreakInput =
      new DigitalInput(CoralGroundIntakeConstants.kIntakeBeamBreakDIO);

  public CoralIntakeIOTalonFX() {
    var motorConfig = CoralGroundIntakeConstants.intakeMotorConfig;
    PhoenixUtil.applyMotorConfigs(intakeMotor, motorConfig, 2);

    var linmotorConfig = CoralGroundIntakeConstants.linearMotorconfig;
    PhoenixUtil.applyMotorConfigs(linearSlideMotor, linmotorConfig, 2);

    BaseStatusSignal.setUpdateFrequencyForAll(
        CoralGroundIntakeConstants.updateFrequency,
        intakeMotorVoltage,
        intakeMotorVelocity,
        intakeMotorStatorCurrent,
        intakeMotorSupplyCurrent,
        intakeMotorTemperature,
        intakeMotorReferenceSlope,
        passthroughMotorVoltage,
        passthroughMotorVelocity,
        passthroughMotorStatorCurrent,
        passthroughMotorSupplyCurrent,
        passthroughMotorTemperature,
        passthroughMotorReferenceSlope);
    intakeMotor.optimizeBusUtilization();
    linearSlideMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(CoralIntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        intakeMotorVoltage,
        intakeMotorVelocity,
        intakeMotorStatorCurrent,
        intakeMotorSupplyCurrent,
        intakeMotorTemperature,
        intakeMotorReferenceSlope,
        passthroughMotorVoltage,
        passthroughMotorVelocity,
        passthroughMotorStatorCurrent,
        passthroughMotorSupplyCurrent,
        passthroughMotorTemperature,
        passthroughMotorReferenceSlope);
    inputs.intakeMotorVoltage = intakeMotorVoltage.getValueAsDouble();
    inputs.intakeMotorVelocity = intakeMotorVelocity.getValueAsDouble();
    inputs.intakeMotorStatorCurrent = intakeMotorStatorCurrent.getValueAsDouble();
    inputs.intakeMotorSupplyCurrent = intakeMotorSupplyCurrent.getValueAsDouble();
    inputs.intakeMotorTemperature = intakeMotorTemperature.getValueAsDouble();
    inputs.intakeMotorReferenceSlope = intakeMotorReferenceSlope.getValueAsDouble();

    inputs.linearMotorVoltage = passthroughMotorVoltage.getValueAsDouble();
    inputs.linearMotorVelocity = passthroughMotorVelocity.getValueAsDouble();
    inputs.linearMotorStatorCurrent = passthroughMotorStatorCurrent.getValueAsDouble();
    inputs.linearMotorSupplyCurrent = passthroughMotorSupplyCurrent.getValueAsDouble();
    inputs.linearMotorTemperature = passthroughMotorTemperature.getValueAsDouble();
    inputs.linearMotorReferenceSlope = passthroughMotorReferenceSlope.getValueAsDouble();

    inputs.isBeamBroken = !beamBreakInput.get();
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  @Override
  public void setIntakeVelocity(double velocity) {
    if (CoralGroundIntakeConstants.kCGIMotorMotionMagic) {
      intakeMotor.setControl(motionMagicIntakeRequest.withVelocity(velocity));
    } else {
      intakeMotor.setControl(intakeRequest.withVelocity(velocity));
    }
  }

  @Override
  public void setLinearMotorVoltage(double voltage) {
    linearSlideMotor.setVoltage(voltage);
  }

  @Override
  public void setLinearMotorVelocity(double velocity) {
    if (CoralGroundIntakeConstants.kLinearSlideMotorMotionMagic) {
      linearSlideMotor.setControl(motionMagicLinearSlideRequest.withVelocity(velocity));
    } else {
      linearSlideMotor.setControl(linearSlideRequest.withVelocity(velocity));
    }
  }

  @Override
  public void off() {
    intakeMotor.setControl(new NeutralOut());
    linearSlideMotor.setControl(new NeutralOut());
  }

  @Override
  public boolean isBeamBroken() {
    return !beamBreakInput.get();
  }

  @Override
  public TalonFX getIntakeMotor() {
    return intakeMotor;
  }

  @Override
  public VoltageOut getIntakeVoltageRequest() {
    return intakeVoltageReq;
  }

  @Override
  public TalonFX getLinearMotor() {
    return linearSlideMotor;
  }

  @Override
  public VoltageOut getLinearVoltageRequest() {
    return linearSlideReq;
  }
}
