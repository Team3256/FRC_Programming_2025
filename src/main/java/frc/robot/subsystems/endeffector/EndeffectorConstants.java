// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.units.measure.AngularVelocity;

public interface EndeffectorConstants {
  public static final int kWristMotorID = 0;
  public static final TalonFXConfiguration kWristMotorConfig = new TalonFXConfiguration();

  public static final int kRollerMotorID = 0;
  public static final TalonFXConfiguration kRollerMotorConfig = new TalonFXConfiguration();

  public static final boolean kUseMotionMagic = true;
  public static final double kStatusSignalUpdateFrequency = 50.0; // Hz
  public static final boolean kUseFOC = true;
  public static final int kFlashConfigRetries = 5;

  public static final AngularVelocity kCoralSpeed = RotationsPerSecond.of(0.0);
  public static final AngularVelocity kAlgaeSpeed = RotationsPerSecond.of(0.0);
  public static final double[] kBranchPosition = {0.0, 0.0, 0.0, 0.0};
  public static final double kAlgaeL2L3Position = 0.0;
  public static final double kAlgaeL3L4Position = 0.0;
  public static final double kProcessorPosition = 0.0;

  public static class SimulationConstants {
    public static final double kRollerGearRatio = 1.0;
    public static final double kWristGearRatio = 1.0;
  }

  public static final boolean kSysIDOnWrist = true;
}
