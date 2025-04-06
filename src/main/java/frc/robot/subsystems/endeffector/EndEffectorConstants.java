// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.AngularVelocity;

public final class EndEffectorConstants {
  public static final boolean kUseFOC = true;
  public static final int algaeMotorID = 45;
  public static final int coralMotorID = 43;

  public static final AngularVelocity l1Velocity = RotationsPerSecond.of(50.0);
  public static final AngularVelocity l2l3Velocity = RotationsPerSecond.of(32);
  public static final double l4Voltage = 2.592;

  // algae first then coral
  public static final AngularVelocity sourceVelocity = RotationsPerSecond.of(30);

  public static final AngularVelocity algaeIntakeVelocity = RotationsPerSecond.of(50);
  public static final AngularVelocity algaeOuttakeVelocity = RotationsPerSecond.of(-100);

  public static TalonFXConfiguration algaeMotorConfigs =
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
  public static TalonFXConfiguration coralMotorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs().withKS(0).withKV(.118).withKA(0).withKP(.2).withKI(0).withKD(0))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(1600)
                  .withMotionMagicCruiseVelocity(0))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(80)
                  .withSupplyCurrentLimit(60)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLowerTime(1)
                  .withSupplyCurrentLowerLimit(40));

  public static final CANdiConfiguration canDiConfigs =
      new CANdiConfiguration()
          .withDigitalInputs(
              new DigitalInputsConfigs()
                  .withS1CloseState(S1CloseStateValue.CloseWhenFloating)
                  .withS2CloseState(S2CloseStateValue.CloseWhenFloating));

  public static final int candiID = 21;

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
