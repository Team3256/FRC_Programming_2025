// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.slapdown;

import static edu.wpi.first.units.Units.*;


import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.sim.SimMechs;
import org.littletonrobotics.junction.LoggedRobot;

public class CoralSlapdownIOSim extends CoralSlapdownIOTalonFX {

  private final SingleJointedArmSim armSimModel =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          CoralSlapdownConstants.Sim.simGearing,
          CoralSlapdownConstants.Sim.jkGMetersSquared,
          CoralSlapdownConstants.Sim.coralSlapdownLength.in(Meters),
          CoralSlapdownConstants.Sim.minAngle.getRadians(),
          CoralSlapdownConstants.Sim.maxAngle.getRadians(),
          true,
          CoralSlapdownConstants.Sim.startingAngle.getRadians());

  private TalonFXSimState coralSlapdownSimState;
 

  public CoralSlapdownIOSim() {
    super();
    coralSlapdownSimState = super.getMotor().getSimState();
    coralSlapdownSimState.Orientation = ChassisReference.Clockwise_Positive;
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    coralSlapdownSimState = super.getMotor().getSimState();
    coralSlapdownSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    armSimModel.setInputVoltage(coralSlapdownSimState.getMotorVoltage());
    armSimModel.update(LoggedRobot.defaultPeriodSecs);
    coralSlapdownSimState.setRawRotorPosition(
        Units.radiansToRotations(armSimModel.getAngleRads()) * CoralSlapdownConstants.Sim.simGearing);
    coralSlapdownSimState.setRotorVelocity(
        Units.radiansToRotations(armSimModel.getVelocityRadPerSec()) * CoralSlapdownConstants.Sim.simGearing);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSimModel.getCurrentDrawAmps()));


    coralSlapdownSimState.setRotorVelocity(
        RadiansPerSecond.of(armSimModel.getVelocityRadPerSec()).in(RotationsPerSecond));
    super.updateInputs(inputs);
    SimMechs.getInstance().updateArm(Radians.of(armSimModel.getAngleRads()));
  }
}
