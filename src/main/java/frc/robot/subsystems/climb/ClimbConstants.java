// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class ClimbConstants {
  public static final int kClimbMotorID = 0;
  public static final double kMaxCurrent = 30;
  public static final double kStowPos = 0;
  public static final double kIntakePos = 90;
  public static final double kLowVoltage = 0.1;
  public static final double kDegreesPerMotorRotation = 360;
  public static final double kMotorTicksPerRotation = 2048;
  public static final double kUpdateFrequency = 0;
  public static final boolean kUseMotionMagic = false;
  public static final double kClimbLength = 3;
  public static final double kClimbMass = 1;
  public static final double kClimbStartingHeight = 10;

  public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(new Slot0Configs().withKS(0).withKV(0).withKP(0).withKI(0).withKD(0))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(400)
                  .withMotionMagicCruiseVelocity(100))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(80));

  public static final class sim {
    public static final double simGearing = 0;
    public static final Distance climbLength = Inches.of(kClimbLength);
    public static final Mass climbMass = Kilograms.of(kClimbMass);
    public static final double jkGMetersSquared = 1.2922967095;

    public static final Rotation2d minAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d maxAngle = Rotation2d.fromDegrees(360);
    public static final Rotation2d startingAngle = Rotation2d.fromDegrees(90);
    public static final Distance startingHeight = Inches.of(kClimbStartingHeight);
  }
}
