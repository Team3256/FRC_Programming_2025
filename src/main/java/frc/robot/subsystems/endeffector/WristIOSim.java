// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import org.littletonrobotics.junction.LoggedRobot;

public class WristIOSim extends WristIOTalonFX {
  private TalonFXSimState motorSim;
  private final DCMotor motorModel;

  public WristIOSim() {
    super();
    this.motorSim = super.getMotor().getSimState();
    this.motorModel =
        (EndeffectorConstants.kUseFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1))
            .withReduction(EndeffectorConstants.SimulationConstants.kRollerGearRatio);
  }

  public void updateInputs(WristIOInputs inputs) {
    // Update battery voltage
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    double rps =
        motorModel.getSpeed(
                motorModel.getTorque(motorSim.getTorqueCurrent()), motorSim.getMotorVoltage())
            / 2
            * Math.PI; // Convert to RPS
    motorSim.setRotorVelocity(rps);
    motorSim.addRotorPosition(rps * LoggedRobot.defaultPeriodSecs);

    // Update battery voltage (after the effects of physics models)
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            // XXX: not sure the difference between this and getting the current
            // from the motor sim
            motorModel.getCurrent(rps, motorSim.getMotorVoltage())));
    super.updateInputs(inputs);

    // TODO: Vis
  }
}
