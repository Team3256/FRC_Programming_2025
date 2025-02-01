// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.slapdown;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface CoralSlapdownIO {
  @AutoLog
  public static class ArmIOInputs {
    public double coralSlapdownMotorVoltage = 0.0;
    public double coralSlapdownVelocity = 0.0;
    public double coralSlapdownMotorPosition = 0.0;
    public double coralSlapdownStatorCurrent = 0.0;
    public double coralSlapdownMotorSupplyCurrent = 0.0;

    public double coralSlapdownEncoderAbsolutePosition = 0.0;
    public double coralSlapdownEncoderPosition = 0.0;
    public double coralSlapdownEncoderVelocity = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setPosition(Angle position, AngularVelocity velocity) {}

  public default void setPosition(Angle position) {}

  public default void setVoltage(Voltage voltage) {}

  public default TalonFX getMotor() {
    return new TalonFX(0);
  }



  public default void off() {}
}
