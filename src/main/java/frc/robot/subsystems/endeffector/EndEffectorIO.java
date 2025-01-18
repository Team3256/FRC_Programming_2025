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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  @AutoLog
  public static class EndEffectorIOInputs {
    public Voltage algaeMotorVoltage = Volts.of(0);
    public AngularVelocity algaeMotorVelocity = RotationsPerSecond.of(0);
    public Current algaeMotorStatorCurrent = Amps.of(0);
    public Current algaeMotorSupplyCurrent = Amps.of(0);

    public Voltage coralMotorVoltage = Volts.of(0);
    public AngularVelocity coralMotorVelocity = RotationsPerSecond.of(0);
    public Current coralMotorStatorCurrent = Amps.of(0);
    public Current coralMotorSupplyCurrent = Amps.of(0);

    public boolean leftBeamBreak = false;
    public boolean rightBeamBreak = false;
  }

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setAlgaeVoltage(double voltage) {}

  public default void setAlgaeVelocity(AngularVelocity velocity) {}

  public default void setCoralVoltage(double voltage) {}

  public default void setCoralVelocity(AngularVelocity velocity) {}

  public default TalonFX getAlgaeMotor() {
    return new TalonFX(0);
  }

  public default TalonFX getCoralMotor() {
    return new TalonFX(0);
  }

  public default CANdi getCandi() {
    return new CANdi(0);
  }

  public default void off() {}
}
