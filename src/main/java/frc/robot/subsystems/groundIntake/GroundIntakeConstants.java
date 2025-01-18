package frc.robot.subsystems.groundIntake;

import static edu.wpi.first.units.Units.*;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class GroundIntakeConstants {

    
  public static final int kMotorID = 0; //Change constants later
  public static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration();
  public static final int groundIntakeMotorEncoderId = 0; //Change constants later

  // max value is 8, min is 0

  /* Misc(From Arm constants, will change in accordance to ground intake later) */
  public static final boolean kUseFOC = false;
  public static final boolean kUseMotionMagic = false; // idk
  public static final double kStatusSignalUpdateFrequency = 50.0;
  public static final int kFlashConfigRetries = 5;

  public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(0)
                  .withKV(3)
                  .withKP(100)
                  .withKI(0)
                  .withKD(0)
                  .withKG(10)
                  .withGravityType(GravityTypeValue.Arm_Cosine) // Original 0.145
              )
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(400)
                  .withMotionMagicCruiseVelocity(50))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(120))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackRemoteSensorID(39)
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                  .withSensorToMechanismRatio(1)
                  .withRotorToSensorRatio(168));

  public static final CANcoderConfiguration cancoderConfiguration =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withMagnetOffset(Rotations.of(0))
                  .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                  .withAbsoluteSensorDiscontinuityPoint(Rotations.of(1)));

  public static final class Sim {
    public static final double simGearing = 168;
    public static final double kGearRatio = 50;

    
    public static final double groundIntakekV = 5;
    public static final double groundIntakekA = 1;
    public static final double minHeightMeters = 0.5;
    public static final double maxHeightMeters = 3;
    public static final boolean simulateGravity = false;
    public static final double startingHeightMeters = 0.5;
    public static final double measurementStdDevs = 0.1;
  }

}
