package frc.robot.subsystems.algae;

//import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.DisableSubsystem;


public class Algae extends DisableSubsystem{
    //private TalonFX algaeRollerMotor = new TalonFX(AlgaeConstants.algaeRollerMotorID);
    private AlgaeIO intakeIO;
    //private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Algae(boolean disabled){
        super(disabled);
        this.intakeIO = intakeIO;
    }
    
    @Override
    public void periodic() {
        super.periodic();
        
    }


    public Command setVoltage(double algaeRollerMotorVoltage){
        return this.run(
        () -> {
            intakeIO.setIntakeVoltage(algaeRollerMotorVoltage);
        });

    }
    public Command setVelocity(double algaeRollerMotorVoltage){
        return this.run(
        () -> {
            intakeIO.setIntakeVelocity(algaeRollerMotorVoltage);
    });

}

    public void simulationPeriodic() {
      
    }
    public boolean isBeamBroken() {
        return intakeIO.isBeamBroken();
    }
}