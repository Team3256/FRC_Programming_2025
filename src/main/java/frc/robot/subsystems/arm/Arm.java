// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import org.json.simple.JSONObject;
import org.littletonrobotics.junction.Logger;
import org.warriorb.lib.custom.DisableSubsystem;
import org.warriorb.lib.misc.MathUtil;

public class Arm extends DisableSubsystem {

  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged armIOAutoLogged = new ArmIOInputsAutoLogged();

  private final Gson GSON = new GsonBuilder().create();

  private Map<String, ArrayList<Map<String, Double>>> loadedTrajs = new HashMap<>();

  private Iterator<Map<String, Double>> trajIterator = null;

  private ArrayList<Map<String, Double>> selectedTraj = null;
  public final Trigger reachedPosition = new Trigger(() -> isAtPosition());
  private Angle requestedPosition = Rotations.of(0.0);

  public Arm(boolean enabled, ArmIO armIO) {
    super(enabled);

    this.armIO = armIO;
    loadAllTraj();
  }

  @Override
  public void periodic() {
    super.periodic();
    armIO.updateInputs(armIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), armIOAutoLogged);

    if (trajIterator != null && trajIterator.hasNext()) {
      armIO.setPosition(
          Radians.of(trajIterator.next().get("position")),
          RadiansPerSecond.of(trajIterator.next().get("velocity")));
    } else if (selectedTraj != null) {
      trajIterator = selectedTraj.iterator();
    }
  }

  public Command runTraj(String trajName) {
    return selectTraj(trajName)
        .andThen(
            this.run(
                () -> {
                  System.out.println(trajIterator.next().get("position"));
                  if (trajIterator != null && trajIterator.hasNext()) {
                    armIO.setPosition(
                        Radians.of(trajIterator.next().get("position")),
                        RadiansPerSecond.of(trajIterator.next().get("velocity")));
                  } else if (selectedTraj != null) {
                    trajIterator = selectedTraj.iterator();
                  }
                }));
  }

  private Command selectTraj(String trajName) {
    return this.runOnce(
        () -> {
          System.out.println(loadedTrajs);
          selectedTraj = loadedTrajs.get(trajName + ".json");
          trajIterator = selectedTraj.iterator();
        });
  }

  public void loadAllTraj() {

    File file = new File(Filesystem.getDeployDirectory(), "arm_traj");
    for (File f : file.listFiles()) {
      try {
        var reader = new BufferedReader(new FileReader(f));
        String str = reader.lines().reduce("", (a, b) -> a + b);
        reader.close();
        loadedTrajs.put(f.getName(), (ArrayList) GSON.fromJson(str, JSONObject.class).get("data"));
      } catch (Exception e) {
        e.printStackTrace();
      }
    }

    //    System.out.println(o.toString());
  }

  public Command setPosition(Angle position) {
    return this.run(
        () -> {
          armIO.setPosition(position);
          requestedPosition = position;
        });
  }

  public Command setVoltage(Voltage voltage) {
    return this.run(() -> armIO.setVoltage(voltage));
  }

  public Command toRightReefLevel(int level) {
    return this.setPosition(ArmConstants.reefRightPositions[level]);
  }

  public Command toLeftReefLevel(int level) {
    return this.setPosition(ArmConstants.reefLeftPositions[level]);
  }

  public Command toRightDealgaeLevel() {
    return this.setPosition(ArmConstants.dealgaeRightPosition);
  }

  public Command toLeftDealgaeLevel(int level) {
    return this.setPosition(ArmConstants.dealgaeLeftPosition);
  }

  public Command toRightSourceLevel() {
    return this.setPosition(ArmConstants.sourceRightPositions);
  }

  public Command toLeftSourceLevel() {
    return this.setPosition(ArmConstants.sourceLeftPositions);
  }

  public boolean isAtPosition() {
    return MathUtil.epsilonEquals(
        armIOAutoLogged.armEncoderAbsolutePosition, requestedPosition.in(Rotations), 0.01);
  }

  public Command toHome() {
    return this.setPosition(ArmConstants.homePosition);
  }

  public Command off() {

    return this.runOnce(armIO::off);
  }
}
