// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils.generics;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public interface SingleMotorConstants {
  public static final int kMotorID = 0;
  public static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration();
  public static final boolean kUseMotionMagic = true;
  public static final double kStatusSignalUpdateFrequency = 50.0; // Hz

  public static final boolean kUseFOC = true;
  public static final int kFlashConfigRetries = 5;

  public static class SimulationConstants {
    public static final double kGearRatio = 1.0;
  }
}
