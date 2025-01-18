package frc.robot.subsystems.groundIntake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.sim.SimMechs;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

public class GroundIntakeIOSim extends GroundIntakeIOTalonFX {

  private TalonFXSimState motorSim;

  private final ElevatorSim groundIntakeSim = 
    new ElevatorSim(
      GroundIntakeConstants.Sim.groundIntakekV, //kV constant
      GroundIntakeConstants.Sim.groundIntakekA,
      DCMotor.getKrakenX60(0), 
      GroundIntakeConstants.Sim.minHeightMeters,
      GroundIntakeConstants.Sim.maxHeightMeters, 
      GroundIntakeConstants.Sim.simulateGravity, 
      GroundIntakeConstants.Sim.startingHeightMeters, 
      GroundIntakeConstants.Sim.measurementStdDevs
    );


  public GroundIntakeIOSim() {
    super();
    this.motorSim = super.getMotor().getSimState();
  }

  @Override
  public void updateInputs(GroundIntakeIOInputs inputs) {
    motorSim = super.getMotor().getSimState();
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    groundIntakeSim.setInputVoltage(motorSim.getMotorVoltage());
    groundIntakeSim.update(LoggedRobot.defaultPeriodSecs);
    motorSim.setRawRotorPosition(
        groundIntakeSim.getPositionMeters() * GroundIntakeConstants.Sim.kGearRatio);
    motorSim.setRotorVelocity(
        groundIntakeSim.getVelocityMetersPerSecond()
            * ElevatorConstants.SimulationConstants.kGearRatio);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(groundIntakeSim.getCurrentDrawAmps()));
    super.updateInputs(inputs);

    Logger.recordOutput("/GroundIntakeSim/positionMeters", groundIntakeSim.getPositionMeters());
    Logger.recordOutput(
        "/GroundIntakeSim/velocityMetersPerSecond", groundIntakeSim.getVelocityMetersPerSecond());
    SimMechs.getInstance().updateElevator(Meters.of(groundIntakeSim.getPositionMeters()));
  }
}
