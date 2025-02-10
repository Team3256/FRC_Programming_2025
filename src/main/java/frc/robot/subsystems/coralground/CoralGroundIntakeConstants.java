// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.coralground;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class CoralGroundIntakeConstants {
  // CAN bus IDs
  public static final int kCGIMotorID = 0;

  // Voltage
  public static final double kCGIMotorVoltage = 0.0;

  // Motion magic enable/disable default values
  public static boolean kCGIMotorMotionMagic = false;

  // time
  public static final double kBeamBreakDelayTime = 0;

  public static final int kIntakeBeamBreakDIO = 0;
  public static double updateFrequency = 0.0;

  public static final TalonFXConfiguration intakeMotorConfig =
      new TalonFXConfiguration()
          .withSlot0(new Slot0Configs().withKS(100).withKP(100).withKI(100).withKD(100).withKV(100))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(
                      NeutralModeValue
                          .Brake) // when no voltage is applied, motor will resist movement (brake)
                  .withInverted(
                      InvertedValue
                          .Clockwise_Positive)) // motor will move in a clockwise dir w/ positive
          // voltage
          .withMotionMagic( // woohoo pay to win
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(0.0)
                  .withMotionMagicCruiseVelocity(0.0)
                  .withMotionMagicJerk(0.0))
          .withCurrentLimits( // prevents frying/destruction of motors
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(0.0));
}
