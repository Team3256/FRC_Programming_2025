package frc.robot.subsystems.slapdown;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class SlapdownConstants {

    // CAN bus IDs
  public static final int kSlapdownMotorID = 0;

  // Voltages
  public static final double kSlapdownMotorVoltage = 0;

  // Motion magic enable/disable default values
  public static boolean kRSlapdownMotionMagic = false;

  public static double updateFrequency = 50;

public static final TalonFXConfiguration slapdownMotorConfig = 
      new TalonFXConfiguration()
          .withSlot0(new Slot0Configs().withKS(100).withKV(100).withKP(100).withKI(200)).withKV(100)
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
                  .withStatorCurrentLimit(60));
}

    

