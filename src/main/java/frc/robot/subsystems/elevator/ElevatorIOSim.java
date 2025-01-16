// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.sim.SimMechs;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim extends ElevatorIOTalonFX {
  private TalonFXSimState motorSim;
  //  private final DCMotor motorModel;
  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          ElevatorConstants.kUseFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1),
          ElevatorConstants.SimulationConstants.kGearRatio,
          ElevatorConstants.SimulationConstants.kCarriageMass.in(Kilograms),
          ElevatorConstants.SimulationConstants.kDrumRadius.in(Meters),
          ElevatorConstants.SimulationConstants.kMinHeight.in(Meters),
          ElevatorConstants.SimulationConstants.kMaxHeight.in(Meters),
          ElevatorConstants.SimulationConstants.kSimulateGravity,
          ElevatorConstants.SimulationConstants.kStartingHeight.in(Meters));

  public ElevatorIOSim() {
    super();
    this.motorSim = super.getMotor().getSimState();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    motorSim = super.getMotor().getSimState();
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    elevatorSim.setInputVoltage(motorSim.getMotorVoltage());
    elevatorSim.update(LoggedRobot.defaultPeriodSecs);
    motorSim.setRawRotorPosition(
        elevatorSim.getPositionMeters() * ElevatorConstants.SimulationConstants.kGearRatio);
    motorSim.setRotorVelocity(
        elevatorSim.getVelocityMetersPerSecond()
            * ElevatorConstants.SimulationConstants.kGearRatio);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    super.updateInputs(inputs);

    Logger.recordOutput("/ElevatorSim/positionMeters", elevatorSim.getPositionMeters());
    Logger.recordOutput(
        "/ElevatorSim/velocityMetersPerSecond", elevatorSim.getVelocityMetersPerSecond());
    SimMechs.getInstance().updateElevator(Meters.of(elevatorSim.getPositionMeters()));
  }
}
