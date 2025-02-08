package frc.robot.subsystems.algae;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.utils.PhoenixUtil;


public class AlgaeIOTalonFX implements AlgaeIO {
    
    private TalonFX algaeRollerMotor = new TalonFX(AlgaeConstants.algaeRollerMotorID);
    final VelocityVoltage velocityVoltageAlgaeRequest = new VelocityVoltage(null);
    final MotionMagicVelocityVoltage motionMagicAlgaeRequest = new MotionMagicVelocityVoltage(null);

    private TalonFX algaeSlapdownMotor = new TalonFX(AlgaeConstants.algaeRollerMotorID);
    final VelocityVoltage velocityVoltageSlapdownRequest = new VelocityVoltage(null);
    final MotionMagicVelocityVoltage motionMagicSlapdownRequest = new MotionMagicVelocityVoltage(null);

    private final StatusSignal<Current> algaeRollerMotorSupply = algaeRollerMotor.getSupplyCurrent();
    private final StatusSignal<Current> algaeRollerMotorStator = algaeRollerMotor.getStatorCurrent();
    private final StatusSignal<AngularVelocity> algaeRollerMotorVelocity = algaeRollerMotor.getVelocity();
    private final StatusSignal<Voltage> algaeRollerMotorVoltage = algaeRollerMotor.getMotorVoltage();
    private final StatusSignal<Temperature> algaeRollerMotorTemperature = algaeRollerMotor.getDeviceTemp();
    private final StatusSignal<Double> algaeRollerMotorReferenceScope = algaeRollerMotor.getClosedLoopReferenceSlope();

    private final StatusSignal<Current> algaeSlapdownMotorSupply = algaeSlapdownMotor.getSupplyCurrent();
    private final StatusSignal<Current> algaeSlapdownMotorStator = algaeSlapdownMotor.getStatorCurrent();
    private final StatusSignal<AngularVelocity> algaeSlapdownMotorVelocity = algaeSlapdownMotor.getVelocity();
    private final StatusSignal<Voltage> algaeSlapdownMotorVoltage = algaeSlapdownMotor.getMotorVoltage();
    private final StatusSignal<Temperature> algaeSlapdownMotorTemperature = algaeSlapdownMotor.getDeviceTemp();
    private final StatusSignal<Double> algaeSlapdownMotorReferenceScope = algaeSlapdownMotor.getClosedLoopReferenceSlope();

    private final DigitalInput beamBreakInput = new DigitalInput(AlgaeConstants.beamBreakDIO);

    public AlgaeIOTalonFX(){

        var algaeConfigs = new TalonFXConfiguration();
        PhoenixUtil.applyMotorConfigs(algaeRollerMotor, algaeConfigs, 2);

        BaseStatusSignal.setUpdateFrequencyForAll(
            algaeRollerMotorSupply,
            algaeRollerMotorStator,
            algaeRollerMotorVelocity,
            algaeRollerMotorVoltage,
            algaeRollerMotorTemperature,
            algaeRollerMotorReferenceScope,

            algaeSlapdownMotorSupply,
            algaeSlapdownMotorStator,
            algaeSlapdownMotorVelocity,
            algaeSlapdownMotorVoltage,
            algaeSlapdownMotorTemperature,
            algaeSlapdownMotorReferenceScope
        );
    }
    @Override
    public void updateInputs(AlgaeInputs inputs){
        BaseStatusSignal.refreshAll(
            algaeRollerMotorSupply,
            algaeRollerMotorStator,
            algaeRollerMotorVelocity,
            algaeRollerMotorVoltage,
            algaeRollerMotorTemperature,
            algaeRollerMotorReferenceScope,

            algaeSlapdownMotorSupply,
            algaeSlapdownMotorStator,
            algaeSlapdownMotorVelocity,
            algaeSlapdownMotorVoltage,
            algaeSlapdownMotorTemperature,
            algaeSlapdownMotorReferenceScope
            );
        inputs.algaeRollerMotorSupply = algaeRollerMotorSupply.getValueAsDouble();
        inputs.algaeRollerMotorStator = algaeRollerMotorStator.getValueAsDouble();
        inputs.algaeRollerMotorTemperature = algaeRollerMotorTemperature.getValueAsDouble();
        inputs.algaeRollerMotorVelocity = algaeRollerMotorVelocity.getValueAsDouble();
        inputs.algaeRollerMotorVoltage = algaeRollerMotorVoltage.getValueAsDouble();
        
        inputs.algaeSlapdownMotorSupply = algaeSlapdownMotorSupply.getValueAsDouble();
        inputs.algaeSlapdownMotorStator = algaeSlapdownMotorStator.getValueAsDouble();
        inputs.algaeSlapdownMotorTemperature = algaeSlapdownMotorTemperature.getValueAsDouble();
        inputs.algaeSlapdownMotorVelocity = algaeSlapdownMotorVelocity.getValueAsDouble();
        inputs.algaeSlapdownMotorVoltage = algaeSlapdownMotorVoltage.getValueAsDouble();
        
        inputs.isBeamBroken = beamBreakInput.get();


    }

    public void setIntakeVelocity(double velocity){


    }

    public void setIntakeVoltage(double voltage){
        algaeRollerMotor.setVoltage(voltage);

    }

}
