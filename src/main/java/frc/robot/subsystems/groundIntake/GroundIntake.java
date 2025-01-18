package frc.robot.subsystems.groundIntake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.utils.DisableSubsystem;
import org.littletonrobotics.junction.Logger;

public class GroundIntake extends DisableSubsystem {
    private final GroundIntakeIO groundIntakeIO;
    private final GroundIntakeIOInputsAutoLogged groundIntakeIOAutoLogged = new GroundIntakeIOInputsAutoLogged();
    private final SysIdRoutine m_sysIdRoutine;

    public GroundIntake(boolean enabled, GroundIntakeIO groundIntakeIO) {
        super(enabled);
        this.groundIntakeIO = groundIntakeIO;
        m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Seconds), // Use default ramp rate (1 V/s)
                Volts.of(6), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) ->
                    groundIntakeIO
                        .getMotor()
                        .setControl(groundIntakeIO.getVoltageRequest().withOutput(volts.in(Volts))),
                null,
                this));
    }

    @Override
    public void periodic() 
    {
        super.periodic();
        groundIntakeIO.updateInputs(groundIntakeIOAutoLogged);
        Logger.processInputs(this.getClass().getSimpleName(), groundIntakeIOAutoLogged);
    }

    public Command setPosition(double position) 
    {
        return this.run(() -> groundIntakeIO.setPosition(position * GroundIntakeConstants.SimulationConstants.kGearRatio));
    }

    public Command off() 
    {
        return this.runOnce(groundIntakeIO::off);
    }

    public Command zero() 
    {
        return this.runOnce(groundIntakeIO::zero);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
      }
    
      public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
      }
}


