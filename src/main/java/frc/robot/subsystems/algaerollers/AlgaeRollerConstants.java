// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.algaerollers;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;

public final class AlgaeRollerConstants {
  public static final boolean kUseFOC = true;
  public static final int algaeMotorID = 35;

  public static final double intakeVoltage = 8;

  public static final double l1Voltage = 2;
  public static final double processorVoltage = -3;

  // algae first then coral

  public static TalonFXConfiguration algaeRollerMotorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs().withKS(0).withKV(.12).withKA(0).withKP(.3).withKI(0).withKD(0))
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
                  .withStatorCurrentLimit(40));

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
