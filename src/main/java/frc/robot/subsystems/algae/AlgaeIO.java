// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

//import org.littletonrobotics.junction.AutoLog;

// import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {

  @AutoLog
  public static class AlgaeInputs{

    public double algaeRollerMotorVoltage = 0.0;
    public double algaeRollerMotorVelocity = 0.0;
    public double algaeRollerMotorTemperature = 0.0;
    public double algaeRollerMotorStator = 0.0;
    public double algaeRollerMotorSupply = 0.0;
    public double algaeRollerMotorReferenceSlope = 0.0;

    public double algaeSlapdownMotorVoltage = 0.0;
    public double algaeSlapdownMotorVelocity = 0.0;
    public double algaeSlapdownMotorTemperature = 0.0;
    public double algaeSlapdownMotorStator = 0.0;
    public double algaeSlapdownMotorSupply = 0.0;
    public double algaeSlapdownMotorReferenceSlope = 0.0;

    public boolean isBeamBroken = false;

  }
  
  public default void updateInputs(AlgaeInputs inputs) {}

  public default void setIntakeVelocity(double velocity){}

  public default void setIntakeVoltage(double voltage){}

  public default void setSlapdownVelocity(double velocity){}

  public default void setSlapdownVoltage(double voltage){}

  public default boolean isBeamBroken(){
    return false;
  };

  

}
