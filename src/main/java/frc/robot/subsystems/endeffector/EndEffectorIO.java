// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  @AutoLog
  public static class EndEffectorIOInputs {
    public double algaeMotorVoltage = 0;
    public double algaeMotorVelocity = 0;
    public double algaeMotorStatorCurrent = 0;
    public double algaeMotorSupplyCurrent = 0;

    public double coralMotorVoltage = 0;
    public double coralMotorVelocity = 0;
    public double coralMotorStatorCurrent = 0;
    public double coralMotorSupplyCurrent = 0;

    public boolean coralBeamBreak = false;
    public boolean algaeBeamBreak = false;
  }

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setAlgaeVoltage(double voltage) {}

  public default void setAlgaeVelocity(AngularVelocity velocity) {}

  public default void setCoralVoltage(double voltage) {}

  public default void setCoralVelocity(AngularVelocity velocity) {}

  public default void setAlgaeVoltage(double voltage, boolean override) {}

  public default void setAlgaeVelocity(AngularVelocity velocity, boolean override) {}

  public default void setCoralVoltage(double voltage, boolean override) {}

  public default void setCoralVelocity(AngularVelocity velocity, boolean override) {}

  public default TalonFX getAlgaeMotor() {
    return new TalonFX(0);
  }

  public default TalonFX getCoralMotor() {
    return new TalonFX(0);
  }

  public default CANdi getCandi() {
    return new CANdi(0);
  }

  public default void algaeOff() {}

  public default void coralOff() {}
}
