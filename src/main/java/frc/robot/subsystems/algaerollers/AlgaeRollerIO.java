// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.algaerollers;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeRollerIO {
  @AutoLog
  public static class AlgaeRollerIOInputs {
    public Voltage algaeMotorVoltage = Volts.of(0);
    public AngularVelocity algaeMotorVelocity = RotationsPerSecond.of(0);
    public Current algaeMotorStatorCurrent = Amps.of(0);
    public Current algaeMotorSupplyCurrent = Amps.of(0);
  }

  public default void updateInputs(AlgaeRollerIOInputs inputs) {}

  public default void setAlgaeVoltage(double voltage) {}

  public default TalonFX getAlgaeRollerMotor() {
    return new TalonFX(0);
  }

  public default void algaeOff() {}
}
