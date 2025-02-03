// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.warriorb.lib.subsystems;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public abstract class DisableSubsystem extends SubsystemBase {
  private class StackJumper {
    private static boolean traceTime = false;

    public static String getCallerMethodName() {
      if (traceTime) {
        double d = HALUtil.getFPGATime();
        String methodName =
            StackWalker.getInstance()
                .walk(stream -> stream.skip(2).findFirst().get())
                .getMethodName();
        System.out.println(
            "Time to get method name: " + (HALUtil.getFPGATime() - d) + " ns for " + methodName);
        return methodName;
      } else {
        return StackWalker.getInstance()
            .walk(stream -> stream.skip(2).findFirst().get())
            .getMethodName();
      }
    }
  }

  public final boolean disabled;

  public DisableSubsystem(boolean enable) {
    super();
    this.disabled = !enable;
  }

  @Override
  public Command run(Runnable action) {
    return disabled
        ? Commands.none()
        : super.run(action).withName(StackJumper.getCallerMethodName());
  }

  @Override
  public Command runOnce(Runnable action) {
    return disabled
        ? Commands.none()
        : super.runOnce(action).withName(StackJumper.getCallerMethodName() + ".runOnce");
  }

  @Override
  public Command startEnd(Runnable action, Runnable end) {
    return disabled
        ? Commands.none()
        : super.startEnd(action, end).withName(StackJumper.getCallerMethodName() + ".startEnd");
  }

  @Override
  public Command runEnd(Runnable action, Runnable end) {
    return disabled
        ? Commands.none()
        : super.runEnd(action, end).withName(StackJumper.getCallerMethodName() + ".runEnd");
  }

  @Override
  public Command defer(Supplier<Command> supplier) {
    return disabled
        ? Commands.none()
        : super.defer(supplier).withName(StackJumper.getCallerMethodName() + ".defer");
  }

  @Override
  public void setDefaultCommand(Command defaultCommand) {
    super.setDefaultCommand(disabled ? Commands.none() : defaultCommand);
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        this.getClass().getSimpleName() + "/CurrentCommand",
        this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "none");
  }

  @Override
  public void simulationPeriodic() {
    this.periodic();
  }
}
