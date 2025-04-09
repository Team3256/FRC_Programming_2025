// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2StateValue;
import com.ctre.phoenix6.sim.CANdiSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final SendableChooser<Boolean> s1Closed =
      new SendableChooser<>() {
        {
          addOption("Algae Inside", true);
          addOption("No Algae", false);
        }
      };
  private final SendableChooser<Boolean> s2Closed =
      new SendableChooser<>() {
        {
          addOption("Coral Inside", true);
          addOption("No Coral", false);
        }
      };

  public EndEffectorIOSim() {
    super();
    algaeMotorSim = super.getAlgaeMotor().getSimState();
    coralMotorSim = super.getCoralMotor().getSimState();
    candiSim = super.getCandi().getSimState();
    s1Closed.onChange(this::updateCandiS1);
    s1Closed.setDefaultOption("No Algae", false);
    SmartDashboard.putData("S1", s1Closed);
    s2Closed.onChange(this::updateCandiS2);
    s2Closed.setDefaultOption("No Coral", false);
    SmartDashboard.putData("S2", s2Closed);
  }

  private void updateCandiS1(boolean beamBroken) {
    candiSim.setS1State(beamBroken ? S1StateValue.Floating : S1StateValue.Low);
  }

  private void updateCandiS2(boolean beamBroken) {
    candiSim.setS2State(beamBroken ? S2StateValue.Floating : S2StateValue.Low);
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
