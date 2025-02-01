// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs; 
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class AlgaeConstants {
  // wait do i have to put the k in front of them
  public static final int algaeRollerMotorID = 0;

  public static final double algaeRollerMotorVoltage = 0.0;

  public static final double beamBreakDelayTime = 0.0;
  public static final int beamBreakDIO = 0;
  
  public static final TalonFXConfiguration algaeRollerMotorConfig = new TalonFXConfiguration()
  .withSlot0(new Slot0Configs().withKP(0).withKI(0).withKV(0).withKS(0))
  .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(0).withStatorCurrentLimitEnable(true))
  .withMotionMagic(new MotionMagicConfigs().withMotionMagicAcceleration(0).withMotionMagicJerk(0))
  // can you check if it is clockwise/counter 
  .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.Clockwise_Positive));
}
