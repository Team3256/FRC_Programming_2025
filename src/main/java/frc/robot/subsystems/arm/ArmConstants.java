// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class ArmConstants {
  public static final int armMotorId = 42;

  public static final int armMotorEncoderId = 40;

  // max value is 8, min is 0

  /* Misc */
  public static final boolean kUseFOC = false;
  public static final boolean kUseMotionMagic = true; // idk
  public static final int flashConfigRetries = 5;

  public static final Angle maxRotations = Rotations.of(2);

  public static final Angle[] reefRightPositions = {
    Rotations.of(0.32), Rotations.of(0.32), Rotations.of(.343)
  };

  public static final Angle[] reefLeftPositions = {
    Rotations.of(0.18), Rotations.of(0.18), Rotations.of(0.157)
  };

  public static final Angle[] dealgaeRightPosition = {Rotations.of(.43), Rotations.of(.361)};
  public static final Angle[] dealgaeLeftPosition = {Rotations.of(.07), Rotations.of(.139)};

  public static final Angle sourceRightPositions = Rotations.of(.66);
  public static final Angle sourceLeftPositions = Rotations.of(.84);

  public static final Angle bargeLeftPosition = Rotations.of(.17);
  public static final Angle bargeRightPosition = Rotations.of(.33);

  public static final Angle homePosition = Rotations.of(.25);

  public static final double safeRightPosition = .45;
  public static final double safeLeftPosition = .05;

  public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(0.18)
                  .withKV(17)
                  .withKP(100)
                  .withKI(0)
                  .withKD(0)
                  .withKA(.7)
                  .withKG(.291)
                  .withGravityType(GravityTypeValue.Arm_Cosine) // Original 0.145
              )
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicJerk(16)
                  .withMotionMagicAcceleration(4.5)
                  .withMotionMagicCruiseVelocity(.65))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(80))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  //                  .withFeedbackRemoteSensorID(armMotorEncoderId)
                  .withSensorToMechanismRatio(142.22)
                  .withRotorToSensorRatio(1));

  public static final CANcoderConfiguration cancoderConfiguration =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withMagnetOffset(Rotations.of(0))
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                  .withMagnetOffset(-0.158122481)
                  .withAbsoluteSensorDiscontinuityPoint(Rotations.of(1)));

  public static final class Sim {
    public static final double simGearing = 142.22;

    public static final Distance armLength = Inches.of(22);
    public static final Mass armMass = Kilograms.of(2);
    public static final double jkGMetersSquared = 1.2922967095;

    public static final Rotation2d minAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d maxAngle = Rotation2d.fromDegrees(360);
    public static final Rotation2d startingAngle = Rotation2d.fromDegrees(0.25);
  }
}
