// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.Date;
import java.util.function.Consumer;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TunableSlot0Configs extends Tunable<Slot0Configs> {

  private LoggedNetworkNumber kP = null;
  private LoggedNetworkNumber kI = null;
  private LoggedNetworkNumber kD = null;
  private LoggedNetworkNumber kS = null;
  private LoggedNetworkNumber kV = null;
  private LoggedNetworkNumber kA = null;
  private LoggedNetworkNumber kG = null;
  // Otherwise it would be an arm (non-static)
  private LoggedNetworkBoolean GravityTypeIsElevator = null; // = GravityTypeValue.Elevator_Static;
  // False = UseVelocitySign
  // True = UseClosedLoopSign
  private LoggedNetworkBoolean StaticFeedforwardSignUseClosedLoop = null;

  private TunableSlot0Configs(
      Slot0Configs value,
      Date tunedDate,
      String author,
      String notes,
      Constants.RobotType robotType) {
    super(value, tunedDate, author, notes, robotType);
  }

  public TunableSlot0Configs toLive(String key) {
    kP = new LoggedNetworkNumber("/Tunable/" + key + "/kP", value.kP);
    kI = new LoggedNetworkNumber("/Tunable/" + key + "/kI", value.kI);
    kD = new LoggedNetworkNumber("/Tunable/" + key + "/kD", value.kD);
    kS = new LoggedNetworkNumber("/Tunable/" + key + "/kS", value.kS);
    kV = new LoggedNetworkNumber("/Tunable/" + key + "/kV", value.kV);
    kA = new LoggedNetworkNumber("/Tunable/" + key + "/kA", value.kA);
    kG = new LoggedNetworkNumber("/Tunable/" + key + "/kG", value.kG);
    GravityTypeIsElevator =
        new LoggedNetworkBoolean(
            "/Tunable/" + key + "/GravityType",
            value.GravityType == GravityTypeValue.Elevator_Static);
    StaticFeedforwardSignUseClosedLoop =
        new LoggedNetworkBoolean(
            "/Tunable/" + key + "/StaticFeedforwardSign",
            value.StaticFeedforwardSign == StaticFeedforwardSignValue.UseClosedLoopSign);
    return this;
  }

  private Slot0Configs getFromLive() {
    if (kP != null
        && kI != null
        && kD != null
        && kS != null
        && kV != null
        && kA != null
        && kG != null
        && GravityTypeIsElevator != null
        && StaticFeedforwardSignUseClosedLoop != null) {
      Slot0Configs slot0Configs = new Slot0Configs();
      slot0Configs.kP = kP.get();
      slot0Configs.kI = kI.get();
      slot0Configs.kD = kD.get();
      slot0Configs.kS = kS.get();
      slot0Configs.kV = kV.get();
      slot0Configs.kA = kA.get();
      slot0Configs.kG = kG.get();
      slot0Configs.GravityType =
          GravityTypeIsElevator.get()
              ? GravityTypeValue.Elevator_Static
              : GravityTypeValue.Arm_Cosine;
      slot0Configs.StaticFeedforwardSign =
          StaticFeedforwardSignUseClosedLoop.get()
              ? StaticFeedforwardSignValue.UseClosedLoopSign
              : StaticFeedforwardSignValue.UseVelocitySign;
      return slot0Configs;
    }
    return null;
  }

  public Slot0Configs get() {
    Slot0Configs slot0Configs = getFromLive();
    if (slot0Configs != null) {
      return slot0Configs;
    }
    return super.get();
  }

  public Trigger onChange(Consumer<Slot0Configs> consumer) {
    if (kP == null) {
      throw new IllegalStateException("onChange cannot be called on a non-live Tunable");
    }
    return new Trigger(() -> value.kP != kP.get())
        .or(() -> value.kI != kI.get())
        .or(() -> value.kD != kD.get())
        .or(() -> value.kS != kS.get())
        .or(() -> value.kV != kV.get())
        .or(() -> value.kA != kA.get())
        .or(() -> value.kG != kG.get())
        .or(
            () ->
                value.GravityType
                    != (GravityTypeIsElevator.get()
                        ? GravityTypeValue.Elevator_Static
                        : GravityTypeValue.Arm_Cosine))
        .or(
            () ->
                value.StaticFeedforwardSign
                    != (StaticFeedforwardSignUseClosedLoop.get()
                        ? StaticFeedforwardSignValue.UseClosedLoopSign
                        : StaticFeedforwardSignValue.UseVelocitySign))
        .onTrue(
            Commands.runOnce(
                () -> {
                  value = getFromLive();
                  consumer.accept(value);
                }));
  }
}
