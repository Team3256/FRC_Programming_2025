// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import frc.robot.subsystems.arm.ArmConstants;

public final class ElevatorConstants {
  public static final int kMotorID = 22;

  public static final int kEncoderAID = 23;
  public static final int kEncoderBID = 24;

  public static final TalonFXConfiguration kMotorConfig =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(0.1)
                  .withKV(1.5)
                  .withKP(1.8)
                  .withKI(0)
                  .withKD(0)
                  .withKG(.6)
                  .withGravityType(GravityTypeValue.Elevator_Static) // Original 0.145
              )
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(20)
                  .withMotionMagicCruiseVelocity(7))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(80));
  public static final boolean kUseMotionMagic = true;
  public static final double kStatusSignalUpdateFrequency = 50.0; // Hz

  public static final boolean kUseFOC = true;
  public static final int kFlashConfigRetries = 5;

  public static final int kEncoderATeethCount = 29;
  public static final int kEncoderBTeethCount = 31;

  public static final CANcoderConfiguration kEncoderAConfig =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withAbsoluteSensorDiscontinuityPoint(Rotations.of(1))
                  .withMagnetOffset(Rotations.of(0))
                  .withSensorDirection(SensorDirectionValue.Clockwise_Positive));
  public static final CANcoderConfiguration kEncoderBConfig =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withAbsoluteSensorDiscontinuityPoint(Rotations.of(1))
                  .withMagnetOffset(Rotations.of(0))
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

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
