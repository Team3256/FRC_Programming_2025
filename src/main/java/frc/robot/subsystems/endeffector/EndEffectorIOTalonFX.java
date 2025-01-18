// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class EndEffectorIOTalonFX implements EndEffectorIO {
  private final TalonFX algaeMotor = new TalonFX(EndEffectorConstants.algaeMotorID);

  private final VelocityVoltage algaeVelocityRequest =
      new VelocityVoltage(0).withSlot(0).withEnableFOC(EndEffectorConstants.kUseFOC);

  private final StatusSignal<Voltage> algaeMotorVoltage = algaeMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> algaeMotorVelocity = algaeMotor.getVelocity();
  private final StatusSignal<Current> algaeMotorStatorCurrent = algaeMotor.getStatorCurrent();
  private final StatusSignal<Current> algaeMotorSupplyCurrent = algaeMotor.getSupplyCurrent();

  private final TalonFX coralMotor = new TalonFX(EndEffectorConstants.coralMotorID);
  final VelocityVoltage velocityRequestFollower =
      new VelocityVoltage(0).withSlot(0).withEnableFOC(EndEffectorConstants.kUseFOC);

  private final StatusSignal<Voltage> coralMotorVoltage = coralMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> coralMotorVelocity = coralMotor.getVelocity();
  private final StatusSignal<Current> coralMotorStatorCurrent = coralMotor.getStatorCurrent();
  private final StatusSignal<Current> coralMotorSupplyCurrent = coralMotor.getSupplyCurrent();


  public EndEffectorIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        algaeMotor,
        EndEffectorConstants.algaeMotorConfigs,
        EndEffectorConstants.flashConfigRetries);

    PhoenixUtil.applyMotorConfigs(
        coralMotor,
        EndEffectorConstants.coralMotorConfigs,
        EndEffectorConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        EndEffectorConstants.updateFrequency,
        algaeMotorVoltage,
        algaeMotorVelocity,
        algaeMotorStatorCurrent,
        algaeMotorSupplyCurrent,
        coralMotorVoltage,
        coralMotorVelocity,
        coralMotorStatorCurrent,
        coralMotorSupplyCurrent);
    algaeMotor.optimizeBusUtilization();
    coralMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        algaeMotorVoltage,
        algaeMotorVelocity,
        algaeMotorStatorCurrent,
        algaeMotorSupplyCurrent,
        coralMotorVoltage,
        coralMotorVelocity,
        coralMotorStatorCurrent,
        coralMotorSupplyCurrent);
    inputs.algaeMotorVoltage = algaeMotorVoltage.getValue();
    inputs.algaeMotorVelocity = algaeMotorVelocity.getValue();
    inputs.algaeMotorStatorCurrent = algaeMotorStatorCurrent.getValue();
    inputs.algaeMotorSupplyCurrent = algaeMotorSupplyCurrent.getValue();
    inputs.coralMotorVoltage = coralMotorVoltage.getValue();
    inputs.coralMotorVelocity = coralMotorVelocity.getValue();
    inputs.coralMotorStatorCurrent = coralMotorStatorCurrent.getValue();
    inputs.coralMotorSupplyCurrent = coralMotorSupplyCurrent.getValue();
  }

  @Override
  public void setAlgaeVoltage(double voltage) {
    algaeMotor.setVoltage(voltage);
  }

  @Override
  public void setAlgaeVelocity(AngularVelocity velocity) {
    algaeMotor.setControl(algaeVelocityRequest.withVelocity(velocity));
  }

  @Override
  public void setCoralVoltage(double voltage) {
    coralMotor.setVoltage(voltage);
  }

  @Override
  public void setCoralVelocity(AngularVelocity velocity) {
    coralMotor.setControl(velocityRequestFollower.withVelocity(velocity));
  }

  @Override
  public void off() {
    algaeMotor.setControl(new NeutralOut());
    coralMotor.setControl(new NeutralOut());
  }

  @Override
  public TalonFX getAlgaeMotor() {
    return algaeMotor;
  }

  @Override
  public TalonFX getCoralMotor() {
    return coralMotor;
  }
}
