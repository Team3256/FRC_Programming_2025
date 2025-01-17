// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class ElevatorConstants {
  public static final int kMotorID = 22;
  public static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration();
  public static final boolean kUseMotionMagic = true;
  public static final double kStatusSignalUpdateFrequency = 50.0; // Hz

  public static final boolean kUseFOC = true;
  public static final int kFlashConfigRetries = 5;

  // Coral positions
  // Please tune
  public static final Angle[] kReefPositions = {
    Rotations.of(0.0), Rotations.of(0.0), Rotations.of(0.0), Rotations.of(0.0)
  };

  public static class SimulationConstants {
    public static final Mass kCarriageMass = Kilograms.of(10);
    public static final double kGearRatio = 50;
    public static final Distance kDrumRadius = Meters.of(1);
    public static final Distance kMaxHeight = Meters.of(10);
    public static final boolean kSimulateGravity = true;
    // Distance from pivot to ground when elevator is stowed??
    // TODO: this is wrong
    // XXX?????? is this even how you're supposed to use it?
    public static final Distance kStartingHeight = Inches.of(7.7);
    public static final Distance kMinHeight = kStartingHeight;
    public static final Distance kWheelRadius = Inches.of(1.0);
    // Elevator extension length
    public static final Distance[] kReefPositions = {
      Inches.of(18 - 7.07).minus(kStartingHeight),
      Inches.of(47.625 - 3).minus(kStartingHeight),
      Inches.of(31.875 - 3).minus(kStartingHeight),
      Inches.of(72 - 7.07).minus(kStartingHeight)
    };
  }
}
