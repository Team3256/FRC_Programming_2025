// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class ArmIOTalonFX implements ArmIO {

  private final TalonFX armMotor = new TalonFX(ArmConstants.armMotorId);
  private final PositionVoltage positionRequest =
      new PositionVoltage(0).withSlot(0).withEnableFOC(ArmConstants.kUseFOC);
  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withSlot(0).withEnableFOC(ArmConstants.kUseFOC);
  private final VoltageOut voltageReq = new VoltageOut(0);

  private final StatusSignal<Voltage> armMotorVoltage = armMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> armMotorVelocity = armMotor.getVelocity();
  private final StatusSignal<Angle> armMotorPosition = armMotor.getPosition();
  private final StatusSignal<Current> armMotorStatorCurrent = armMotor.getStatorCurrent();
  private final StatusSignal<Current> armMotorSupplyCurrent = armMotor.getSupplyCurrent();

  private final CANcoder cancoder = new CANcoder(ArmConstants.armMotorEncoderId);

  private final StatusSignal<Angle> cancoderAbsolutePosition = cancoder.getAbsolutePosition();
  private final StatusSignal<Angle> cancoderPosition = cancoder.getPosition();
  private final StatusSignal<AngularVelocity> cancoderVelocity = cancoder.getVelocity();

  public ArmIOTalonFX() {

    PhoenixUtil.applyMotorConfigs(
        armMotor, ArmConstants.motorConfigs, ArmConstants.flashConfigRetries);

    PhoenixUtil.applyCancoderConfig(
        cancoder, ArmConstants.cancoderConfiguration, ArmConstants.flashConfigRetries);

    //    BaseStatusSignal.setUpdateFrequencyForAll(
    //        ArmConstants.updateFrequency,
    //        armMotorVoltage,
    //        armMotorVelocity,
    //        armMotorPosition,
    //        armMotorStatorCurrent,
    //        armMotorSupplyCurrent);

    //    armMotor.optimizeBusUtilization();
    //    cancoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        armMotorVoltage,
        armMotorVelocity,
        armMotorPosition,
        armMotorStatorCurrent,
        armMotorSupplyCurrent,
        cancoderAbsolutePosition,
        cancoderPosition,
        cancoderVelocity);
    inputs.armMotorVoltage = armMotorVoltage.getValue().in(Volt);
    inputs.armMotorVelocity = armMotorVelocity.getValue().in(RotationsPerSecond);
    inputs.armMotorPosition = armMotorPosition.getValue().in(Rotations);
    inputs.armMotorStatorCurrent = armMotorStatorCurrent.getValue().in(Amps);
    inputs.armMotorSupplyCurrent = armMotorSupplyCurrent.getValue().in(Amps);
    inputs.armEncoderPosition = cancoderPosition.getValue().in(Rotations);
    inputs.armEncoderVelocity = cancoderVelocity.getValue().in(RotationsPerSecond);
    inputs.armEncoderAbsolutePosition = cancoderAbsolutePosition.getValue().in(Rotations);
  }

  @Override
  public void setPosition(Angle position) {
    if (ArmConstants.kUseMotionMagic) {
      armMotor.setControl(motionMagicRequest.withPosition(position));
    } else {
      armMotor.setControl(positionRequest.withPosition(position));
    }
  }

  @Override
  public void setPosition(double position) {
    if (ArmConstants.kUseMotionMagic) {
      armMotor.setControl(motionMagicRequest.withPosition(position));
    } else {
      armMotor.setControl(positionRequest.withPosition(position));
    }
  }

  @Override
  public void setPosition(Angle position, AngularVelocity velocity) {
    armMotor.setControl(positionRequest.withPosition(position).withVelocity(velocity));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    armMotor.setVoltage(voltage.in(Volt));
  }

  @Override
  public void off() {
    armMotor.setControl(new NeutralOut());
  }

  @Override
  public TalonFX getMotor() {
    return armMotor;
  }

  @Override
  public CANcoder getEncoder() {
    return cancoder;
  }

  @Override
  public void resetPosition(Angle angle) {
    armMotor.setPosition(angle);
  }
}
