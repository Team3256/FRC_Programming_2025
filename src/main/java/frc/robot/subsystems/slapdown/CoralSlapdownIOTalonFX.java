// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.slapdown;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.Map;

public class CoralSlapdownIOTalonFX implements CoralSlapdownIO {

  private final TalonFX armMotor = new TalonFX(CoralSlapdownConstants.coralSlapdownMotorID);
  private final PositionVoltage positionRequest =
      new PositionVoltage(0).withSlot(0).withEnableFOC(CoralSlapdownConstants.kUseFOC);
  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withSlot(0).withEnableFOC(CoralSlapdownConstants.kUseFOC);
  private final VoltageOut voltageReq = new VoltageOut(0);

  private final StatusSignal<Voltage> armMotorVoltage = armMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> armMotorVelocity = armMotor.getVelocity();
  private final StatusSignal<Angle> armMotorPosition = armMotor.getPosition();
  private final StatusSignal<Current> armMotorStatorCurrent = armMotor.getStatorCurrent();
  private final StatusSignal<Current> armMotorSupplyCurrent = armMotor.getSupplyCurrent();

  private final Gson GSON = new GsonBuilder().create();

  private ArrayList<Map<String, Double>> loadedTraj = null;

  private Iterator<Map<String, Double>> trajIterator = null;

  public CoralSlapdownIOTalonFX() {

    PhoenixUtil.applyMotorConfigs(
        armMotor, CoralSlapdownConstants.motorConfigs, CoralSlapdownConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        CoralSlapdownConstants.updateFrequency,
        armMotorVoltage,
        armMotorVelocity,
        armMotorPosition,
        armMotorStatorCurrent,
        armMotorSupplyCurrent);

    armMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        armMotorVoltage,
        armMotorVelocity,
        armMotorPosition,
        armMotorStatorCurrent,
        armMotorSupplyCurrent);
    inputs.coralSlapdownMotorVoltage = armMotorVoltage.getValue().in(Volt);
    inputs.coralSlapdownVelocity = armMotorVelocity.getValue().in(RotationsPerSecond);
    inputs.coralSlapdownMotorPosition = armMotorPosition.getValue().in(Rotations);
    inputs.coralSlapdownStatorCurrent = armMotorStatorCurrent.getValue().in(Amps);
    inputs.coralSlapdownMotorSupplyCurrent = armMotorSupplyCurrent.getValue().in(Amps);
  }

  public void goLoadedTraj() {
    if (trajIterator.hasNext()) {
      Map<String, Double> point = trajIterator.next();
      armMotor.setControl(
          positionRequest
              .withPosition(Radians.of(point.get("position")))
              .withVelocity(RadiansPerSecond.of(point.get("velocity"))));
    } else {
      trajIterator = loadedTraj.iterator();
    }
  }

  @Override
  public void setPosition(Angle position) {
    if (CoralSlapdownConstants.kUseMotionMagic) {
      armMotor.setControl(motionMagicRequest.withPosition(position));
    } else {
      armMotor.setControl(positionRequest.withPosition(position));
    }
  }

  @Override
  public void setPosition(Angle position, AngularVelocity velocity) {
    armMotor.setControl(positionRequest.withPosition(position).withVelocity(velocity));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    armMotor.setVoltage(voltage.in(Volt));
  }

  @Override
  public void off() {
    armMotor.setControl(new NeutralOut());
  }

  @Override
  public TalonFX getMotor() {
    return armMotor;
  }
}
