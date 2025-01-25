// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.CoralGroundIntake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface CoralIntakeIO {
  @AutoLog
  public static class CoralIntakeIOInputs {
    public double intakeMotorVoltage = 0.0;
    public double intakeMotorVelocity = 0.0;
    public double intakeMotorStatorCurrent = 0.0;
    public double intakeMotorSupplyCurrent = 0.0;
    public double intakeMotorTemperature = 0.0;
    public double intakeMotorReferenceSlope = 0.0;

    public double linearMotorVoltage = 0.0;
    public double linearMotorVelocity = 0.0;
    public double linearMotorStatorCurrent = 0.0;
    public double linearMotorSupplyCurrent = 0.0;
    public double linearMotorTemperature = 0.0;
    public double linearMotorReferenceSlope = 0.0;

    public boolean isBeamBroken = false;
  }

  public default void updateInputs(CoralIntakeIOInputs inputs) {}

  public default void setIntakeVoltage(double voltage) {}

  public default void setIntakeVelocity(double velocity) {}

  public default void setLinearMotorVoltage(double voltage) {}

  public default void setLinearMotorVelocity(double velocity) {}

  public default TalonFX getIntakeMotor() {
    return new TalonFX(0);
  }

  public default VoltageOut getIntakeVoltageRequest() {
    return new VoltageOut(0);
  }

  public default TalonFX getLinearMotor() {
    return new TalonFX(0);
  }

  public default VoltageOut getLinearVoltageRequest() {
    return new VoltageOut(0);
  }

  public default void off() {}

  public default boolean isBeamBroken() {
    return false;
  }
}
