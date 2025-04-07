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
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.arm.AlgaeArmIO;
import frc.robot.utils.PhoenixUtil;

public class AlgaeArmTalonFX implements AlgaeArmIO {

  private final TalonFX algaeArmMotor = new TalonFX(AlgaeArmConstants.armMotorId);
  private final PositionVoltage positionRequest =
      new PositionVoltage(0).withSlot(0).withEnableFOC(AlgaeArmConstants.kUseFOC);
  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withSlot(0).withEnableFOC(AlgaeArmConstants.kUseFOC);
  private final VoltageOut voltageReq = new VoltageOut(0);

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

    algaeArmMotor.optimizeBusUtilization();
    //    cancoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(AlgaeArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        armMotorVoltage,
        armMotorVelocity,
        armMotorPosition,
        armMotorStatorCurrent,
        armMotorSupplyCurrent);
    inputs.armMotorVoltage = armMotorVoltage.getValue().in(Volt);
    inputs.armMotorVelocity = armMotorVelocity.getValue().in(RotationsPerSecond);
    inputs.armMotorPosition = armMotorPosition.getValue().in(Rotations);
    inputs.armMotorStatorCurrent = armMotorStatorCurrent.getValue().in(Amps);
    inputs.armMotorSupplyCurrent = armMotorSupplyCurrent.getValue().in(Amps);
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
  public void setPosition(double position) {
    if (AlgaeArmConstants.kUseMotionMagic) {
      algaeArmMotor.setControl(motionMagicRequest.withPosition(position));
    } else {
      algaeArmMotor.setControl(positionRequest.withPosition(position));
    }
  }

  @Override
  public void setPosition(Angle position, AngularVelocity velocity) {
    algaeArmMotor.setControl(positionRequest.withPosition(position).withVelocity(velocity));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    algaeArmMotor.setVoltage(voltage.in(Volt));
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
