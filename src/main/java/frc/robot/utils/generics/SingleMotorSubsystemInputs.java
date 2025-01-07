// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils.generics;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class SingleMotorSubsystemInputs {
  public double motorVoltage = 0.0;
  public double motorVelocity = 0.0;
  public double motorPosition = 0.0;
  public double motorStatorCurrent = 0.0;
  public double motorSupplyCurrent = 0.0;
  public double motorTemperature = 0.0;
  public double motorReferenceSlope = 0.0;
}
