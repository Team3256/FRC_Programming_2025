package frc.robot.subsystems.CoralGroundIntake;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public interface CoralGroundIntakeIO {
    @AutoLog
    public static class coralIntakeIOInputs {
      public double intakeMotorVoltage = 0.0;
      public double intakeMotorVelocity = 0.0;
      public double intakeMotorStatorCurrent = 0.0;
      public double intakeMotorSupplyCurrent = 0.0;
      public double intakeMotorTemperature = 0.0;
      public double intakeMotorReferenceSlope = 0.0;
  
      public double linearMotorVoltage = 0.0;
      public double linearMotorVelocity = 0.0;
      public double linearMotorStatorCurrent = 0.0;
      public double linearMotorSupplyCurrent = 0.0;
      public double linearMotorTemperature = 0.0;
      public double linearMotorReferenceSlope = 0.0;
  
      public boolean isBeamBroken = false;
    }

  public default void updateInputs(coralIntakeIOInputs inputs) {}

  public default void setIntakeVoltage(double voltage) {}

  public default void setIntakeVelocity(double velocity) {}

  public default void setLinearMotorVoltage(double voltage) {}

  public default void setLinearMotorVelocity(double velocity) {}

  public default TalonFX getIntakeMotor() {
    return new TalonFX(0);
  }

  public default VoltageOut getIntakeVoltageRequest() {
    return new VoltageOut(0);
  }

  public default TalonFX getLinearMotor() {
    return new TalonFX(0);
  }

  public default VoltageOut getLinearVoltageRequest() {
    return new VoltageOut(0);
  }

  public default void off() {}

  public default boolean isBeamBroken() {
    return false;
  }
    

    
}
