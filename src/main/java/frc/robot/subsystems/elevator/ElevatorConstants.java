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
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.arm.ArmConstants;

public final class ElevatorConstants {
  public static final int kMotorID = 44;

  public static final int kEncoderAID = 23;
  public static final int kEncoderBID = 24;

  public static final TalonFXConfiguration kMotorConfig =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(.31640625)
                  .withKV(1.428)
                  .withKP(150)
                  .withKD(0)
                  .withKA(.1)
                  .withKG(.158203125)
                  .withGravityType(GravityTypeValue.Elevator_Static) // Original 0.145
              )
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(30.75)
                  .withMotionMagicCruiseVelocity(7.2)
                  .withMotionMagicJerk(0))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(80))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(12)
                  .withRotorToSensorRatio(1));
  public static final boolean kUseMotionMagic = true;
  public static final double kStatusSignalUpdateFrequency = 50.0; // Hz

  public static final boolean kUseFOC = true;
  public static final int kFlashConfigRetries = 5;

  public static final int kEncoderATeethCount = 29;
  public static final int kEncoderBTeethCount = 31;

  public static final Angle armSafePosition = Rotations.of(2);

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

  // L1, L2, L3, L4
  // 3 sigfigs of precision
  public static final Angle[] kReefPositionsPracticeField = {
    Rotations.of(0.7), Rotations.of(1.39), Rotations.of(2.7), Rotations.of(4.875)
  };
  public static final Distance[] kReefPositionsMeters = Constants.branchHeights.distances;

  // L1 -> L4, in order
  public static final Angle[] kReefPositions = {
    convertMetersToRotations(
            FieldConstants.BranchHeights.PRACTICE_FIELD.distances[0].minus(kReefPositionsMeters[0]))
        .plus(kReefPositionsPracticeField[0]),
    convertMetersToRotations(
            FieldConstants.BranchHeights.PRACTICE_FIELD.distances[1].minus(kReefPositionsMeters[1]))
        .plus(kReefPositionsPracticeField[1]),
    convertMetersToRotations(
            FieldConstants.BranchHeights.PRACTICE_FIELD.distances[2].minus(kReefPositionsMeters[2]))
        .plus(kReefPositionsPracticeField[2]),
    convertMetersToRotations(
            FieldConstants.BranchHeights.PRACTICE_FIELD.distances[3].minus(kReefPositionsMeters[3]))
        .plus(kReefPositionsPracticeField[3]),
  };

  public static final Angle[] kDealgaePositions = {Rotations.of(.6), Rotations.of(2.0)};

  public static final Angle sourcePosition = Rotations.of(.15);
  public static final Angle bargePosition = Rotations.of(5);

  public static final Angle homePosition = Rotations.of(.2);
  public static final Angle processorPosition = Rotations.of(2.1); // not real for now
  public static final Angle groundAlgaePosition = Rotations.of(2.4);

  public static class SimulationConstants {
    public static final Mass kCarriageMass = Pounds.of(2.5).plus(ArmConstants.Sim.armMass);
    public static final double kGearRatio = 20;
    public static final Distance kDrumRadius = Inches.of(2);
    public static final Distance kMinHeight = Inches.of(0);
    public static final Distance kMaxHeight = Inches.of(100);
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

  public static Distance convertRotationsToMeters(Angle rotations) {
    return SimulationConstants.kDrumRadius.times(2 * Math.PI * rotations.in(Rotations));
  }

  public static Angle convertMetersToRotations(Distance dist) {
    return Rotations.of(dist.div(SimulationConstants.kDrumRadius.times(2 * Math.PI)).magnitude());
  }
}
