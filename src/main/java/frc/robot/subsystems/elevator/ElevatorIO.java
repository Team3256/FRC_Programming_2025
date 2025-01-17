// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public class ElevatorIOInputs {
    public double motorVoltage = 0.0;
    public double motorVelocity = 0.0;
    public double motorPosition = 0.0;
    public double motorStatorCurrent = 0.0;
    public double motorSupplyCurrent = 0.0;
    public double motorTemperature = 0.0;
    public double motorReferenceSlope = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setPosition(double position) {}

  public default TalonFX getMotor() {
    return new TalonFX(0);
  }

  public default VoltageOut getVoltageRequest() {
    return new VoltageOut(0);
  }

  public default void off() {
    this.getMotor().setControl(new NeutralOut());
  }

  public default void zero() {
    this.getMotor().setPosition(0);
  }
}
