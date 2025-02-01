package frc.robot.subsystems.algae;

import java.util.logging.Logger;

//import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.DisableSubsystem;


public class Algae extends DisableSubsystem{
    //private TalonFX algaeRollerMotor = new TalonFX(AlgaeConstants.algaeRollerMotorID);
    private AlgaeIO intakeIO;
    // edit
    private final Trigger debouncedBeamBreak = new Trigger(null);
    //private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Algae(boolean disabled, AlgaeIO intakeIO){
        super(disabled);
        this.intakeIO = intakeIO;
    }
    
    @Override
    public void periodic() {
        super.periodic();

        //Logger.updateInputs("intake algae", );
        //Logger.processInputs();
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

   // public Command off(){
        //return this.runOnce(intakeIO::off);
   // }
    public void simulationPeriodic() {
      
    }
    public boolean isBeamBroken() {
        return intakeIO.isBeamBroken();
    }
}