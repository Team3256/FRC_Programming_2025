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

    public static double algaeRollerMotorVoltage = 0.0;
    public static double algaeRollerMotorVelocity = 0.0;
    public static double algaeRollerMotorTemperature = 0.0;
    public static double algaeRollerMotorStator = 0.0;
    public static double algaeRollerMotorSupply = 0.0;
    public static double algaeRollerMotorReferenceSlope = 0.0;

    public boolean isBeamBroken = false;

  }
  
  public default void intakeLog(AlgaeInputs inputs) {}

  public default void setIntakeVelocity(double velocity){}

  public default void setIntakeVoltage(double voltage){}

  public default boolean isBeamBroken(){
    return false;
  };

  

}
