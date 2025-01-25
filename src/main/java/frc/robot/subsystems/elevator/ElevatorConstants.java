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
import frc.robot.subsystems.arm.ArmConstants;

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

  public static final Angle[] kDealgaePositions = {Rotations.of(0.0), Rotations.of(0.0)};

  public static final Angle sourcePosition = Rotations.of(0.0);
  public static final Angle bargePosition = Rotations.of(0.0);

  public static final Angle homePosition = Rotations.of(0.0);

  public static class SimulationConstants {
    public static final Mass kCarriageMass = Pounds.of(2.5).plus(ArmConstants.Sim.armMass);
    public static final double kGearRatio = 20;
    public static final Distance kDrumRadius = Inches.of(2);
    public static final Distance kMinHeight = Inches.of(0);
    public static final Distance kMaxHeight = Inches.of(69.422);
    public static final boolean kSimulateGravity = true;
    // Drivebase to minHeight
    public static final Distance kStartingHeight = Inches.of(1.75);
    // Elevator extension length
    // TODO: re calculate this lol. Should be relative to minHeight
    public static final Distance[] kReefPositions = {
      Inches.of(18 - 7.07).minus(kStartingHeight),
      Inches.of(47.625 - 3).minus(kStartingHeight),
      Inches.of(31.875 - 3).minus(kStartingHeight),
      Inches.of(72 - 7.07).minus(kStartingHeight)
    };
  }
}
