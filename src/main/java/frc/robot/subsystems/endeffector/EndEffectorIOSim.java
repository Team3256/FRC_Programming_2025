// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.sim.CANdiSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.sim.SimMechs;
import org.littletonrobotics.junction.LoggedRobot;

public class EndEffectorIOSim extends EndEffectorIOTalonFX {
  private final FlywheelSim algaeSimModel =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              EndEffectorConstants.kUseFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1),
              EndEffectorConstants.SimulationConstants.algaeGearingRatio,
              EndEffectorConstants.SimulationConstants.algaeMomentOfInertia),
          EndEffectorConstants.kUseFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1));
  private final FlywheelSim coralSimModel =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              EndEffectorConstants.kUseFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1),
              EndEffectorConstants.SimulationConstants.coralGearingRatio,
              EndEffectorConstants.SimulationConstants.coralMomentOfInertia),
          EndEffectorConstants.kUseFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1));
  private final TalonFXSimState algaeMotorSim;
  private final TalonFXSimState coralMotorSim;

  private final CANdiSimState candiSim;

  public EndEffectorIOSim() {
    super();
    algaeMotorSim = super.getAlgaeMotor().getSimState();
    coralMotorSim = super.getCoralMotor().getSimState();
    candiSim = super.getCandi().getSimState();
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {

    // Update battery voltage
    algaeMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    coralMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    candiSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    // Update physics models
    algaeSimModel.setInput(algaeMotorSim.getMotorVoltage());
    algaeSimModel.update(LoggedRobot.defaultPeriodSecs);
    coralSimModel.setInput(coralMotorSim.getMotorVoltage());
    coralSimModel.update(LoggedRobot.defaultPeriodSecs);

    double algaeRps = algaeSimModel.getAngularVelocityRPM() / 60;
    algaeMotorSim.setRotorVelocity(algaeRps);
    algaeMotorSim.addRotorPosition(algaeRps * LoggedRobot.defaultPeriodSecs);
    double coralRps = coralSimModel.getAngularVelocityRPM() / 60;
    coralMotorSim.setRotorVelocity(coralRps);
    coralMotorSim.addRotorPosition(coralRps * LoggedRobot.defaultPeriodSecs);

    // Update battery voltage (after the effects of physics models)
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            algaeSimModel.getCurrentDrawAmps(), coralSimModel.getCurrentDrawAmps()));
    super.updateInputs(inputs);

    SimMechs.getInstance()
        .updateEndEffector(
            Degrees.of(
                Math.toDegrees(algaeRps)
                    * LoggedRobot.defaultPeriodSecs
                    * EndEffectorConstants.SimulationConstants.kAngularVelocityScalar),
            Degrees.of(
                Math.toDegrees(coralRps)
                    * LoggedRobot.defaultPeriodSecs
                    * EndEffectorConstants.SimulationConstants.kAngularVelocityScalar));
  }
}
