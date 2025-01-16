// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double armMotorVoltage = 0.0;
    public double armMotorVelocity = 0.0;
    public double armMotorPosition = 0.0;
    public double armMotorStatorCurrent = 0.0;
    public double armMotorSupplyCurrent = 0.0;

    public double armEncoderAbsolutePosition = 0.0;
    public double armEncoderPosition = 0.0;
    public double armEncoderVelocity = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setPosition(Angle position) {}

  public default void setVoltage(Voltage voltage) {}

  public default TalonFX getMotor() {
    return new TalonFX(0);
  }

  public default CANcoder getEncoder() {
    return new CANcoder(0);
  }

  public default void off() {}
}
