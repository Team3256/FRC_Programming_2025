package frc.robot.subsystems.algae;

import java.util.logging.Logger;

//import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.DisableSubsystem;


public class Algae extends DisableSubsystem{
    
    private AlgaeIO intakeIO;
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
   // public Command off(){
        //this.runOnce(intakeIO::off);
   // }


    public Command setIntakeVelocity(double algaeRollerMotorVelocity){
        return this.run(() -> {intakeIO.setIntakeVelocity(algaeRollerMotorVelocity);});
    }

    public Command setIntakeVoltage(double algaeRollerMotorVoltage){
        return this.run(() -> {intakeIO.setIntakeVelocity(algaeRollerMotorVoltage);});
    }

    public Command setSlapdownVelocity(double algaeSlapdownMotorVelocity){
        return this.run(() -> {intakeIO.setSlapdownVelocity(algaeSlapdownMotorVelocity);});
    }

    public Command setSlapdownVoltage(double algaeRollerMotorVoltage){
        return this.run(() -> {intakeIO.setSlapdownVelocity(algaeRollerMotorVoltage);});
    }
    


    public Command setVoltage(double algaeRollerMotorVoltage, double algaeSlapdownMotorVoltage){
        return this.run(
        () -> {
            intakeIO.setIntakeVoltage(algaeRollerMotorVoltage);
            intakeIO.setIntakeVoltage(algaeSlapdownMotorVoltage);
        });


    }
    public Command setVelocity(double algaeRollerMotorVoltage){
        return this.run(
        () -> {
            intakeIO.setIntakeVelocity(algaeRollerMotorVoltage);
        });
 

}
    public Command slapdownAndIntake(double velocity, double voltage) {
        return setIntakeVelocity(0).andThen(setSlapdownVoltage(voltage));
    }

   // public Command off(){
        //return this.runOnce(intakeIO::off);
   // }

    public Command intakeIn (){
        return this.run(
        () -> {
            intakeIO.setSlapdownVelocity(0);
            intakeIO.setIntakeVoltage(AlgaeConstants.algaeRollerMotorVoltage);
            
        });
    }
    public void simulationPeriodic() {
      
    }
    public boolean isBeamBroken() {
        return intakeIO.isBeamBroken();
    }
}