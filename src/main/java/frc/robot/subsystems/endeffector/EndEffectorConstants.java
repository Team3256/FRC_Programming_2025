// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class EndEffectorConstants {
  public static final boolean kUseFOC = true;
  public static int algaeMotorID = 43;
  public static int coralMotorID = 48;
  public static TalonFXConfiguration algaeMotorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs().withKS(0).withKV(0.15).withKA(0).withKP(8).withKI(0).withKD(0.1))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(1600)
                  .withMotionMagicCruiseVelocity(0))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60));
  public static TalonFXConfiguration coralMotorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs().withKS(0).withKV(0.15).withKA(0).withKP(8).withKI(0).withKD(0.1))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(1600)
                  .withMotionMagicCruiseVelocity(0))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60));

  public static final class SimulationConstants {
    public static double coralGearingRatio = 1.0;
    public static double coralMomentOfInertia = 0.0001;
    public static double algaeGearingRatio = 1.0;
    public static double algaeMomentOfInertia = 0.0001;
    // Scale down the angular velocity so we can actually see what is happening
    public static double kAngularVelocityScalar = 0.05;
  }

  public static double updateFrequency = 50.0;

  public static int flashConfigRetries = 5;
}
