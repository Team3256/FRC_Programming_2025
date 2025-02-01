// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.slapdown;

import static edu.wpi.first.units.Units.Rotations;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.Util;
import java.util.ArrayList;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class CoralSlapdown extends DisableSubsystem {

  private final CoralSlapdownIO coralSlapdownIO;
  private final CoralSlapdownIOInputsAutoLogged coralSlapdownIOAutoLogged =
      new CoralSlapdownIOInputsAutoLogged();

  private final Gson GSON = new GsonBuilder().create();

  private ArrayList<Map<String, Double>> selectedTraj = null;
  public final Trigger reachedPosition = new Trigger(() -> isAtPosition());
  private Angle requestedPosition = Rotations.of(0.0);

  public CoralSlapdown(boolean enabled, CoralSlapdownIO armIO) {
    super(enabled);

    this.coralSlapdownIO = armIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    coralSlapdownIO.updateInputs(coralSlapdownIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), coralSlapdownIOAutoLogged);
  }

  public Command setPosition(Angle position) {
    return this.run(
        () -> {
          coralSlapdownIO.setPosition(position);
          requestedPosition = position;
        });
  }

  public Command setVoltage(Voltage voltage) {
    return this.run(() -> coralSlapdownIO.setVoltage(voltage));
  }

  public Command toRightReefLevel(int level) {
    return this.setPosition(CoralSlapdownConstants.reefRightPositions[level]);
  }

  public Command toLeftReefLevel(int level) {
    return this.setPosition(CoralSlapdownConstants.reefLeftPositions[level]);
  }

  public Command toRightDealgaeLevel() {
    return this.setPosition(CoralSlapdownConstants.dealgaeRightPosition);
  }

  public Command toLeftDealgaeLevel(int level) {
    return this.setPosition(CoralSlapdownConstants.dealgaeLeftPosition);
  }

  public Command toRightSourceLevel() {
    return this.setPosition(CoralSlapdownConstants.sourceRightPositions);
  }

  public Command toLeftSourceLevel() {
    return this.setPosition(CoralSlapdownConstants.sourceLeftPositions);
  }

  public boolean isAtPosition() {
    return Util.epsilonEquals(
        coralSlapdownIOAutoLogged.coralSlapdownEncoderAbsolutePosition,
        requestedPosition.in(Rotations),
        0.01);
  }

  public Command toHome() {
    return this.setPosition(CoralSlapdownConstants.homePosition);
  }

  public Command off() {

    return this.runOnce(coralSlapdownIO::off);
  }
}
