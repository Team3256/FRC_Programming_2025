package frc.robot.subsystems.climb;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsytem extends SubsystemBase{
    private final TalonFX climbMotor = new TalonFX(ClimbConstants.kClimbMotorID);

    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(ClimbConstants.kClimbMotorID);

    public ClimbSubsytem() {
        climbMotor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());
    }

    public void setDegrees(double deg) {
        double targetPosition = deg / (ClimbConstants.kDegreesPerMotorRotation) * (ClimbConstants.kMotorTicksPerRotation);
        climbMotor.setControl(new MotionMagicVoltage(targetPosition));
    }

    public void zeroClimb() {
        climbMotor.setControl(new MotionMagicVoltage(ClimbConstants.kLowVoltage));
        while (!isCurrentSpiking()) {}
        climbMotor.setControl(new MotionMagicVoltage(0));
    }


    public boolean isCurrentSpiking() {
        return climbMotor.getSupplyCurrent().getValueAsDouble() > ClimbConstants.kMaxCurrent;
    }

    public Command stowClimb() {
        return new InstantCommand(
            () -> {
                zeroClimb();
                setDegrees(ClimbConstants.kStowPos);
            },
        this);
    }
}
