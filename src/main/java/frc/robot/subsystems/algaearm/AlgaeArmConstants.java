// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.algaearm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class AlgaeArmConstants {
  public static final int armMotorId = 34;

  // max value is 8, min is 0

  /* Misc */
  public static final boolean kUseFOC = false;
  public static final boolean kUseMotionMagic = true; // idk
  public static final int flashConfigRetries = 5;
  public static final double updateFrequency = 50.0;

  public static final Angle homePosition = Rotations.of(.25);
  public static Angle groundAlgaePosition = Rotations.of(.45);
  public static Angle partialDeploy = Rotations.of(.35);

  public static Angle l1Position = Rotations.of(.32);

  public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(.2)
                  .withKV(0.95138)
                  .withKP(100)
                  .withKI(0)
                  .withKD(4)
                  .withKA(0)
                  .withKG(.05)
                  .withGravityType(GravityTypeValue.Arm_Cosine) // Original 0.145
              )
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicJerk(0)
                  .withMotionMagicAcceleration(14)
                  .withMotionMagicCruiseVelocity(2))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(40))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(13.5)
                  .withRotorToSensorRatio(1));

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
