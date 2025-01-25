package frc.robot.subsystems.algae;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


public class AlgaeIOTalonFX implements AlgaeIO {
    
    private TalonFX algaeRollerMotor = new TalonFX(AlgaeConstants.algaeRollerMotorID);
    
    final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(null);
    final MotionMagicVelocityVoltage motionMagicRequest = new MotionMagicVelocityVoltage(null);

    

}
