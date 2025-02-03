// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.warriorb.lib.misc.PhoenixUtil;

public class RollerIOTalonFX implements RollerIO {

  final TalonFX rollerMotor = new TalonFX(RollerConstants.kRollerMotorID);
  // establishing control requests for INTAKE MOTOR, with desired output and pid
  // slot parameters

  private final StatusSignal<Current> rollerMotorStatorCurrent = rollerMotor.getStatorCurrent();
  private final StatusSignal<Voltage> rollerMotorVoltage = rollerMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> rollerMotorVelocity = rollerMotor.getVelocity();

  public RollerIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(rollerMotor, RollerConstants.rollerMotorConfig, 3);
    BaseStatusSignal.setUpdateFrequencyForAll(
        RollerConstants.updateFrequency,
        rollerMotorVoltage,
        rollerMotorVelocity,
        rollerMotorStatorCurrent);
    rollerMotor.optimizeBusUtilization();
    // TalonUtil.applyAndCheckConfiguration(rollerMotor,rollerMotorConfig);
  }

  @Override
  public void setVoltage(double voltage) {
    rollerMotor.setVoltage(voltage);
  }

  @Override
  public void off() {
    rollerMotor.setControl(new NeutralOut());
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    BaseStatusSignal.refreshAll(rollerMotorVoltage, rollerMotorVelocity);

    inputs.rollerMotorVoltage = rollerMotorVoltage.getValueAsDouble();
    inputs.rollerMotorVelocity = rollerMotorVelocity.getValueAsDouble();
    inputs.rollerMotorStatorCurrent = rollerMotorStatorCurrent.getValueAsDouble();
  }
}
