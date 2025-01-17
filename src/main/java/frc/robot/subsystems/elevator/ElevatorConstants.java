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

  public static class SimulationConstants {
    public static final Mass kCarriageMass = Kilograms.of(6.989).plus(ArmConstants.Sim.armMass);
    public static final double kGearRatio = 50;
    public static final Distance kDrumRadius = Inches.of(1.995);
    public static final Distance kMaxHeight = Inches.of(68.424);
    public static final boolean kSimulateGravity = true;
    // Distance from pivot to ground when elevator is stowed??
    // TODO: this is wrong
    // XXX?????? is this even how you're supposed to use it?
    public static final Distance kStartingHeight = Inches.of(7.7);
    public static final Distance kMinHeight = Inches.of(41);

    // Elevator extension length
    public static final Distance[] kReefPositions = {
      Inches.of(18 - 7.07).minus(kStartingHeight),
      Inches.of(47.625 - 3).minus(kStartingHeight),
      Inches.of(31.875 - 3).minus(kStartingHeight),
      Inches.of(72 - 7.07).minus(kStartingHeight)
    };
    // See
    // https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/closed-loop-requests.html#converting-from-meters
    public static final Distance kWheelRadius = kDrumRadius;
  }
}
