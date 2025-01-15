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
import frc.robot.drivers.SimViz;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim extends ElevatorIOTalonFX {
  private TalonFXSimState motorSim;
  private final DCMotor motorModel;
  private final ElevatorSim elevatorSim;

  public ElevatorIOSim() {
    super();
    this.motorSim = super.getMotor().getSimState();
    this.motorModel =
        (ElevatorConstants.kUseFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1))
            .withReduction(ElevatorConstants.SimulationConstants.kGearRatio);
    this.elevatorSim =
        new ElevatorSim(
            motorModel,
            ElevatorConstants.SimulationConstants.kGearRatio,
            ElevatorConstants.SimulationConstants.kCarriageMass.in(Kilograms),
            ElevatorConstants.SimulationConstants.kDrumRadius.in(Meters),
            ElevatorConstants.SimulationConstants.kMinHeight.in(Meters),
            ElevatorConstants.SimulationConstants.kMaxHeight.in(Meters),
            ElevatorConstants.SimulationConstants.kSimulateGravity,
            ElevatorConstants.SimulationConstants.kStartingHeight.in(Meters),
            null);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    // Update battery voltage
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Yes, I could've done .getMotorVoltageMeasure().in(Volts)
    // but this does the exact same thing
    elevatorSim.setInputVoltage(motorSim.getMotorVoltage());
    elevatorSim.update(0.02);

    // Update battery voltage (after the effects of physics models)
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    super.updateInputs(inputs);

    Logger.recordOutput("/ElevatorSim/positionMeters", elevatorSim.getPositionMeters());
    Logger.recordOutput(
        "/ElevatorSim/velocityMetersPerSecond", elevatorSim.getVelocityMetersPerSecond());
    SimViz.getInstance().updateElevatorPosition(Meters.of(elevatorSim.getPositionMeters()));
  }
}
