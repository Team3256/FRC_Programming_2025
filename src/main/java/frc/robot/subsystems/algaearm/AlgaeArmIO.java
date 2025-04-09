// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.algaearm;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeArmIO {
  @AutoLog
  public static class AlgaeArmIOInputs {
    public double algaeArmMotorVoltage = 0.0;
    public double algaeArmMotorVelocity = 0.0;
    public double algaeArmMotorPosition = 0.0;
    public double algaeArmMotorStatorCurrent = 0.0;
    public double algaeArmMotorSupplyCurrent = 0.0;
  }

  public default void updateInputs(AlgaeArmIOInputs inputs) {}

  public default void setPosition(Angle position) {}

  public default void setVoltage(double voltage) {}

  public default TalonFX getMotor() {
    return new TalonFX(0);
  }

  public default void resetPosition(Angle angle) {}

  public default void off() {}
}
