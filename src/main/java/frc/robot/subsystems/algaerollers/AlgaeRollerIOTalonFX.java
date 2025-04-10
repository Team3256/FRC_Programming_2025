// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.algaerollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class AlgaeRollerIOTalonFX implements AlgaeRollerIO {
  private final TalonFX algaeRollerMotor = new TalonFX(AlgaeRollerConstants.algaeMotorID);

  private final VoltageOut algaeVoltageRequest =
      new VoltageOut(0).withEnableFOC(AlgaeRollerConstants.kUseFOC);

  private final StatusSignal<Voltage> algaeMotorVoltage = algaeRollerMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> algaeMotorVelocity = algaeRollerMotor.getVelocity();
  private final StatusSignal<Current> algaeMotorStatorCurrent = algaeRollerMotor.getStatorCurrent();
  private final StatusSignal<Current> algaeMotorSupplyCurrent = algaeRollerMotor.getSupplyCurrent();

  public AlgaeRollerIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        algaeRollerMotor,
        AlgaeRollerConstants.algaeRollerMotorConfigs,
        AlgaeRollerConstants.flashConfigRetries);
    //    BaseStatusSignal.setUpdateFrequencyForAll(
    //        AlgaeRollerConstants.updateFrequency,
    //        algaeMotorVoltage,
    //        algaeMotorVelocity,
    //        algaeMotorStatorCurrent,
    //        algaeMotorSupplyCurrent);
    //    algaeRollerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(AlgaeRollerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        algaeMotorVoltage, algaeMotorVelocity, algaeMotorStatorCurrent, algaeMotorSupplyCurrent);
    inputs.algaeMotorVoltage = algaeMotorVoltage.getValueAsDouble();
    inputs.algaeMotorVelocity = algaeMotorVelocity.getValueAsDouble();
    inputs.algaeMotorStatorCurrent = algaeMotorStatorCurrent.getValueAsDouble();
    inputs.algaeMotorSupplyCurrent = algaeMotorSupplyCurrent.getValueAsDouble();
  }

  @Override
  public void setAlgaeVoltage(double voltage) {
    algaeRollerMotor.setControl(algaeVoltageRequest.withOutput(voltage));
  }

  @Override
  public void algaeOff() {
    algaeRollerMotor.setControl(new NeutralOut());
  }

  @Override
  public TalonFX getAlgaeRollerMotor() {
    return algaeRollerMotor;
  }
}
