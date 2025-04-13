// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.algaearm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class AlgaeArmTalonFX implements AlgaeArmIO {

  private final TalonFX algaeArmMotor = new TalonFX(AlgaeArmConstants.armMotorId);
  private final PositionVoltage positionRequest =
      new PositionVoltage(0).withSlot(0).withEnableFOC(AlgaeArmConstants.kUseFOC);
  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withSlot(0).withEnableFOC(AlgaeArmConstants.kUseFOC);

  private final StatusSignal<Voltage> armMotorVoltage = algaeArmMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> armMotorVelocity = algaeArmMotor.getVelocity();
  private final StatusSignal<Angle> armMotorPosition = algaeArmMotor.getPosition();
  private final StatusSignal<Current> armMotorStatorCurrent = algaeArmMotor.getStatorCurrent();
  private final StatusSignal<Current> armMotorSupplyCurrent = algaeArmMotor.getSupplyCurrent();

  public AlgaeArmTalonFX() {

    PhoenixUtil.applyMotorConfigs(
        algaeArmMotor, AlgaeArmConstants.motorConfigs, AlgaeArmConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        AlgaeArmConstants.updateFrequency,
        armMotorVoltage,
        armMotorVelocity,
        armMotorPosition,
        armMotorStatorCurrent,
        armMotorSupplyCurrent);

    algaeArmMotor.optimizeBusUtilization(4, 0.100);
  }

  @Override
  public void updateInputs(AlgaeArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        armMotorVoltage,
        armMotorVelocity,
        armMotorPosition,
        armMotorStatorCurrent,
        armMotorSupplyCurrent);
    inputs.algaeArmMotorVoltage = armMotorVoltage.getValue().in(Volt);
    inputs.algaeArmMotorVelocity = armMotorVelocity.getValue().in(RotationsPerSecond);
    inputs.algaeArmMotorPosition = armMotorPosition.getValue().in(Rotations);
    inputs.algaeArmMotorStatorCurrent = armMotorStatorCurrent.getValue().in(Amps);
    inputs.algaeArmMotorSupplyCurrent = armMotorSupplyCurrent.getValue().in(Amps);
  }

  @Override
  public void setPosition(Angle position) {
    if (AlgaeArmConstants.kUseMotionMagic) {
      algaeArmMotor.setControl(motionMagicRequest.withPosition(position));
    } else {
      algaeArmMotor.setControl(positionRequest.withPosition(position));
    }
  }

  @Override
  public void setVoltage(double voltage) {
    algaeArmMotor.setVoltage(voltage);
  }

  @Override
  public void off() {
    algaeArmMotor.setControl(new NeutralOut());
  }

  @Override
  public TalonFX getMotor() {
    return algaeArmMotor;
  }

  @Override
  public void resetPosition(Angle angle) {
    algaeArmMotor.setPosition(angle);
  }
}
