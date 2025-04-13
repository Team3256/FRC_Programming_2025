// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class EndEffectorIOTalonFX implements EndEffectorIO {
  private final TalonFX algaeMotor = new TalonFX(EndEffectorConstants.algaeMotorID);

  private final VelocityVoltage algaeVelocityRequest =
      new VelocityVoltage(0).withSlot(0).withEnableFOC(EndEffectorConstants.kUseFOC);

  private final VoltageOut algaeVoltageRequest =
      new VoltageOut(0).withEnableFOC(EndEffectorConstants.kUseFOC);

  private final StatusSignal<Voltage> algaeMotorVoltage = algaeMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> algaeMotorVelocity = algaeMotor.getVelocity();
  private final StatusSignal<Current> algaeMotorStatorCurrent = algaeMotor.getStatorCurrent();
  private final StatusSignal<Current> algaeMotorSupplyCurrent = algaeMotor.getSupplyCurrent();

  private final TalonFX coralMotor = new TalonFX(EndEffectorConstants.coralMotorID);
  final VelocityVoltage coralVelocityRequest =
      new VelocityVoltage(0).withSlot(0).withEnableFOC(EndEffectorConstants.kUseFOC);

  private final VoltageOut coralVoltageRequest =
      new VoltageOut(0).withEnableFOC(EndEffectorConstants.kUseFOC);

  private final StatusSignal<Voltage> coralMotorVoltage = coralMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> coralMotorVelocity = coralMotor.getVelocity();
  private final StatusSignal<Current> coralMotorStatorCurrent = coralMotor.getStatorCurrent();
  private final StatusSignal<Current> coralMotorSupplyCurrent = coralMotor.getSupplyCurrent();

  private final CANdi candi = new CANdi(EndEffectorConstants.candiID);

  private final StatusSignal<Boolean> coralBeamBreak = candi.getS2Closed();
  private final StatusSignal<Boolean> algaeBeamBreak = candi.getS1Closed();

  public EndEffectorIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        algaeMotor,
        EndEffectorConstants.algaeMotorConfigs,
        EndEffectorConstants.flashConfigRetries);

    PhoenixUtil.applyMotorConfigs(
        coralMotor,
        EndEffectorConstants.coralMotorConfigs,
        EndEffectorConstants.flashConfigRetries);

    PhoenixUtil.applyCANdiConfigs(
        candi, EndEffectorConstants.canDiConfigs, EndEffectorConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        EndEffectorConstants.updateFrequency,
        algaeMotorVoltage,
        algaeMotorVelocity,
        algaeMotorStatorCurrent,
        algaeMotorSupplyCurrent,
        coralMotorVoltage,
        coralMotorVelocity,
        coralMotorStatorCurrent,
        coralMotorSupplyCurrent,
        coralBeamBreak,
        algaeBeamBreak);
    algaeMotor.optimizeBusUtilization(4, 0.100);
    coralMotor.optimizeBusUtilization(4, 0.100);
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
        coralMotorSupplyCurrent,
        coralBeamBreak,
        algaeBeamBreak);
    inputs.algaeMotorVoltage = algaeMotorVoltage.getValueAsDouble();
    inputs.algaeMotorVelocity = algaeMotorVelocity.getValueAsDouble();
    inputs.algaeMotorStatorCurrent = algaeMotorStatorCurrent.getValueAsDouble();
    inputs.algaeMotorSupplyCurrent = algaeMotorSupplyCurrent.getValueAsDouble();
    inputs.coralMotorVoltage = coralMotorVoltage.getValueAsDouble();
    inputs.coralMotorVelocity = coralMotorVelocity.getValueAsDouble();
    inputs.coralMotorStatorCurrent = coralMotorStatorCurrent.getValueAsDouble();
    inputs.coralMotorSupplyCurrent = coralMotorSupplyCurrent.getValueAsDouble();

    inputs.coralBeamBreak = coralBeamBreak.getValue();
    inputs.algaeBeamBreak = algaeBeamBreak.getValue();
  }

  @Override
  public void setAlgaeVoltage(double voltage) {
    setAlgaeVoltage(voltage, false);
  }

  @Override
  public void setAlgaeVelocity(AngularVelocity velocity) {
    setAlgaeVelocity(velocity, false);
  }

  @Override
  public void setCoralVoltage(double voltage) {
    setCoralVoltage(voltage, false);
  }

  @Override
  public void setCoralVelocity(AngularVelocity velocity) {
    setCoralVelocity(velocity, false);
  }

  @Override
  public void setAlgaeVoltage(double voltage, boolean override) {
    algaeMotor.setControl(
        algaeVoltageRequest.withOutput(Volts.of(voltage)).withIgnoreHardwareLimits(override));
  }

  @Override
  public void setAlgaeVelocity(AngularVelocity velocity, boolean override) {
    algaeMotor.setControl(
        algaeVelocityRequest.withVelocity(velocity).withIgnoreHardwareLimits(override));
  }

  @Override
  public void setCoralVoltage(double voltage, boolean override) {
    coralMotor.setControl(
        coralVoltageRequest.withOutput(Volts.of(voltage)).withIgnoreHardwareLimits(override));
  }

  @Override
  public void setCoralVelocity(AngularVelocity velocity, boolean override) {
    coralMotor.setControl(
        coralVelocityRequest.withVelocity(velocity).withIgnoreHardwareLimits(override));
  }

  @Override
  public void algaeOff() {
    algaeMotor.setControl(new NeutralOut());
  }

  @Override
  public void coralOff() {
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

  @Override
  public CANdi getCandi() {
    return candi;
  }
}
