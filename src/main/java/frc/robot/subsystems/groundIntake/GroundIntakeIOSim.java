// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

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
import frc.robot.sim.SimMechs;
import org.littletonrobotics.junction.LoggedRobot;

public class GroundIntakeIOSim extends GroundIntakeIOTalonFX {

  private final SingleJointedArmSim groundIntakeSimModel =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          GroundIntakeConstants.Sim.simGearing,
          GroundIntakeConstants.Sim.jkGMetersSquared,
          GroundIntakeConstants.Sim.armLength.in(Meters),
          GroundIntakeConstants.Sim.minAngle.getRadians(),
          GroundIntakeConstants.Sim.maxAngle.getRadians(),
          true,
          GroundIntakeConstants.Sim.startingAngle
              .getRadians()); // Change this model later to be Ground Intake

  private TalonFXSimState groundIntakeSimState;
  private CANcoderSimState cancoderSimState;

  public GroundIntakeIOSim() {
    super();
    groundIntakeSimState = super.getMotor().getSimState();
    cancoderSimState = super.getEncoder().getSimState();
    cancoderSimState.Orientation = ChassisReference.Clockwise_Positive;
    groundIntakeSimState.Orientation = ChassisReference.Clockwise_Positive;
  }

  @Override
  public void updateInputs(GroundIntakeIOInputs inputs) {

    groundIntakeSimState = super.getMotor().getSimState();
    groundIntakeSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    groundIntakeSimModel.setInputVoltage(groundIntakeSimState.getMotorVoltage());
    groundIntakeSimModel.update(LoggedRobot.defaultPeriodSecs);
    groundIntakeSimState.setRawRotorPosition(
        Units.radiansToRotations(groundIntakeSimModel.getAngleRads())
            * GroundIntakeConstants.Sim.simGearing);
    groundIntakeSimState.setRotorVelocity(
        Units.radiansToRotations(groundIntakeSimModel.getVelocityRadPerSec())
            * GroundIntakeConstants.Sim.simGearing);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(groundIntakeSimModel.getCurrentDrawAmps()));

    cancoderSimState = super.getEncoder().getSimState();
    cancoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    cancoderSimState.setRawPosition(Radians.of(groundIntakeSimModel.getAngleRads()).in(Rotations));
    groundIntakeSimState.setRotorVelocity(
        RadiansPerSecond.of(groundIntakeSimModel.getVelocityRadPerSec()).in(RotationsPerSecond));
    super.updateInputs(inputs);
    SimMechs.getInstance().updateArm(Radians.of(groundIntakeSimModel.getAngleRads()));
  }
}
