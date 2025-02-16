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
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.Util;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import org.json.simple.JSONObject;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends DisableSubsystem {

  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged armIOAutoLogged = new ArmIOInputsAutoLogged();

  private final Gson GSON = new GsonBuilder().create();

  private Map<String, ArrayList<Map<String, Double>>> loadedTrajs = new HashMap<>();

  private Iterator<Map<String, Double>> trajIterator = null;

  private ArrayList<Map<String, Double>> selectedTraj = null;
  public final Trigger reachedPosition = new Trigger(this::isAtPosition);
  public final Trigger isSafePosition = new Trigger(this::isSafePosition);
  private final MutAngle requestedPosition = Rotations.of(0.0).mutableCopy();

  public Arm(boolean enabled, ArmIO armIO) {
    super(enabled);

    this.armIO = armIO;
    armIO.resetPosition(Rotations.of(0.25));
    loadAllTraj();
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.recordOutput(
        this.getClass().getSimpleName() + "/requestedPosition", requestedPosition.in(Rotations));
    armIO.updateInputs(armIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), armIOAutoLogged);

    //    if (trajIterator != null && trajIterator.hasNext()) {
    //      armIO.setPosition(
    //          Radians.of(trajIterator.next().get("position")),
    //          RadiansPerSecond.of(trajIterator.next().get("velocity")));
    //    } else if (selectedTraj != null) {
    //      trajIterator = selectedTraj.iterator();
    //    }
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

  public Command setPosition(Supplier<Angle> position, boolean continuous, IntSupplier direction) {
    return this.run(
        () -> {
          requestedPosition.mut_replace(
              continuous
                  ? continuousWrapAtHome(position.get(), direction.getAsInt())
                  : position.get());
          armIO.setPosition(requestedPosition);
        });
  }

  public Command setPosition(Angle position, boolean continuous, int direction) {
    return setPosition(() -> position, continuous, () -> direction);
  }

  public Command setPosition(double position, boolean continuous, int direction) {
    return setPosition(() -> Rotations.of(position), continuous, () -> direction);
  }

  public Command setPosition(DoubleSupplier position, boolean continuous, IntSupplier direction) {
    return setPosition(() -> Rotations.of(position.getAsDouble()), continuous, direction);
  }

  public Command setVoltage(Voltage voltage) {
    return this.run(() -> armIO.setVoltage(voltage));
  }

  public Command toReefLevel(int level, BooleanSupplier rightSide) {
    return setPosition(
        () ->
            rightSide.getAsBoolean()
                ? ArmConstants.reefRightPositions[level]
                : ArmConstants.reefLeftPositions[level],
        true,
        () -> 0);
  }

  public Command toDealgaeLevel(BooleanSupplier rightSide) {
    return this.setPosition(
        () ->
            rightSide.getAsBoolean()
                ? ArmConstants.dealgaeRightPosition
                : ArmConstants.dealgaeLeftPosition,
        true,
        () -> 0);
  }

  @AutoLogOutput
  public boolean isSafePosition() {
    return (armIOAutoLogged.armMotorPosition + 5) % 1 >= 0.15
        && (armIOAutoLogged.armMotorPosition + 5) % 1 <= .35;
  }

  public Command toSourceLevel(BooleanSupplier rightSide) {
    return this.setPosition(
        () ->
            rightSide.getAsBoolean()
                ? ArmConstants.sourceRightPositions
                : ArmConstants.sourceLeftPositions,
        true,
        () -> rightSide.getAsBoolean() ? -1 : 1);
  }

  @AutoLogOutput
  public boolean isAtPosition() {
    return Util.epsilonEquals(
        armIOAutoLogged.armMotorPosition, requestedPosition.in(Rotations), 0.05);
  }

  public Command toHome() {
    return this.setPosition(ArmConstants.homePosition, true, 0);
  }

  public Command toHome(BooleanSupplier preferRightSide) {
    return this.setPosition(
        () -> ArmConstants.homePosition, true, () -> preferRightSide.getAsBoolean() ? -1 : 1);
  }

  public Command off() {
    return this.runOnce(armIO::off);
  }

  public Angle continuousWrapAtHome(Angle angle, int direction) {
    return Rotations.of(continuousWrapAtHome(angle.in(Rotations), direction));
  }

  public double continuousWrapAtHome(double angle, int direction) {
    return continuousWrapAtHome(angle, armIOAutoLogged.armMotorPosition, direction);
  }

  public static double continuousWrapAtHome(
      double reqAbsAngle, double currentAngle, double forcedDirection) {
    // forcedDirection should be nonzero; its sign determines the rotation adjustment.
    // Compute the allowed range for the integer offset.
    int n_min = (int) Math.ceil(-ArmConstants.maxRotations.in(Rotations) - reqAbsAngle);
    int n_max = (int) Math.floor(ArmConstants.maxRotations.in(Rotations) - reqAbsAngle);

    // Compute the short-path candidate.
    int nIdeal = (int) Math.round(currentAngle - reqAbsAngle);
    int nCandidate = Math.min(n_max, Math.max(n_min, nIdeal));

    // Use the sign of forcedDirection to determine the adjustment.
    int adjustment = (int) Math.signum(forcedDirection);

    // Add the adjustment to force the long path.
    int nLong = nCandidate + adjustment;
    nLong = Math.min(n_max, Math.max(n_min, nLong));

    return reqAbsAngle + nLong;
  }
}
