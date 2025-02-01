package frc.robot.subsystems.algae;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;


public class AlgaeIOTalonFX implements AlgaeIO {
    
    private TalonFX algaeRollerMotor = new TalonFX(AlgaeConstants.algaeRollerMotorID);
    
    final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(null);
    final MotionMagicVelocityVoltage motionMagicRequest = new MotionMagicVelocityVoltage(null);

    private final StatusSignal<Current> algaeSupply = algaeRollerMotor.getSupplyCurrent();
    private final StatusSignal<Current> algaeStator = algaeRollerMotor.getStatorCurrent();
    private final StatusSignal<AngularVelocity> algaeVelocity = algaeRollerMotor.getVelocity();
    private final StatusSignal<Voltage> algaeVoltage = algaeRollerMotor.getMotorVoltage();
    private final StatusSignal<Temperature> algaeTemperature = algaeRollerMotor.getDeviceTemp();
    private final StatusSignal<Double> algaeReferenceScope = algaeRollerMotor.getClosedLoopReferenceSlope();

    
    @Override
    public void updateInputs(AlgaeInputs inputs){
        BaseStatusSignal.refreshAll(
            algaeSupply,
            algaeStator,
            algaeVelocity,
            algaeVoltage,
            algaeTemperature,
            algaeReferenceScope
        );

    }

    public void setIntakeVelocity(double velocity){


    }

    public void setIntakeVoltage(double voltage){
        algaeRollerMotor.setVoltage(voltage);

    }

}
