// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

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
import frc.robot.sim.SimMechs;
import org.littletonrobotics.junction.LoggedRobot;

public class ArmIOSim extends ArmIOTalonFX {

  private final SingleJointedArmSim armSimModel =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          ArmConstants.Sim.simGearing,
          ArmConstants.Sim.jkGMetersSquared,
          ArmConstants.Sim.armLength.in(Meters),
          ArmConstants.Sim.minAngle.getRadians(),
          ArmConstants.Sim.maxAngle.getRadians(),
          true,
          ArmConstants.Sim.startingAngle.getRadians());

  private TalonFXSimState armSimState;
  private CANcoderSimState cancoderSimState;

  public ArmIOSim() {
    super();
    armSimState = super.getMotor().getSimState();
    cancoderSimState = super.getEncoder().getSimState();
    cancoderSimState.Orientation = ChassisReference.Clockwise_Positive;
    armSimState.Orientation = ChassisReference.Clockwise_Positive;
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    armSimState = super.getMotor().getSimState();
    armSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    armSimModel.setInputVoltage(armSimState.getMotorVoltage());
    armSimModel.update(LoggedRobot.defaultPeriodSecs);
    armSimState.setRawRotorPosition(
        Units.radiansToRotations(armSimModel.getAngleRads()) * ArmConstants.Sim.simGearing);
    armSimState.setRotorVelocity(
        Units.radiansToRotations(armSimModel.getVelocityRadPerSec()) * ArmConstants.Sim.simGearing);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSimModel.getCurrentDrawAmps()));

    cancoderSimState = super.getEncoder().getSimState();
    cancoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    cancoderSimState.setRawPosition(Radians.of(armSimModel.getAngleRads()).in(Rotations));
    armSimState.setRotorVelocity(
        RadiansPerSecond.of(armSimModel.getVelocityRadPerSec()).in(RotationsPerSecond));
    super.updateInputs(inputs);
    SimMechs.getInstance().updateArm(Radians.of(armSimModel.getAngleRads()));
  }
}
