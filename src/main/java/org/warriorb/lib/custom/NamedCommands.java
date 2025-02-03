// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.warriorb.lib.custom;

import edu.wpi.first.wpilibj2.command.*;
import java.util.Arrays;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public final class NamedCommands {

  public static Command none() {
    return new InstantCommand().withName("None");
  }

  public static Command idle(Subsystem... requirements) {
    return Commands.run(() -> {}, requirements).withName("Idle: " + Arrays.toString(requirements));
  }

  public static Command waitSeconds(double seconds) {
    return new WaitCommand(seconds).withName("Wait " + seconds + "s");
  }

  public static Command waitUntil(BooleanSupplier condition) {
    return new WaitUntilCommand(condition).withName("Wait until " + condition.toString());
  }

  public static Command either(Command onTrue, Command onFalse, BooleanSupplier selector) {
    return new ConditionalCommand(onTrue, onFalse, selector)
        .withName(
            "Either "
                + onTrue.getName()
                + " or "
                + onFalse.getName()
                + " based on "
                + selector.toString());
  }

  public static <K> Command select(Map<K, Command> commands, Supplier<? extends K> selector) {
    return new SelectCommand(commands, selector)
        .withName(
            "Select from "
                + Arrays.toString(commands.values().stream().map(Command::getName).toArray()));
  }

  public static Command defer(Supplier<Command> supplier, Set<Subsystem> requirements) {
    return new DeferredCommand(supplier, requirements)
        .withName("Defer " + supplier.get().getName());
  }

  // public static Command deferredProxy(Supplier<Command> supplier) {
  //   return new ProxyCommand(supplier).withName(supplier.get().getName());
  // }

  public static Command sequence(Command... commands) {
    return new SequentialCommandGroup(commands)
        .withName(
            "Sequence " + Arrays.toString(Arrays.stream(commands).map(Command::getName).toArray()));
  }

  public static Command repeatingSequence(Command... commands) {
    return sequence(commands)
        .repeatedly()
        .withName(
            "Repeating Sequence "
                + Arrays.toString(Arrays.stream(commands).map(Command::getName).toArray()));
  }

  public static Command parallel(Command... commands) {
    return new ParallelCommandGroup(commands)
        .withName(
            "Parallel " + Arrays.toString(Arrays.stream(commands).map(Command::getName).toArray()));
  }

  public static Command race(Command... commands) {
    return new ParallelRaceGroup(commands)
        .withName(
            "Race " + Arrays.toString(Arrays.stream(commands).map(Command::getName).toArray()));
  }

  public static Command deadline(Command deadline, Command... otherCommands) {
    return new ParallelDeadlineGroup(deadline, otherCommands)
        .withName(
            "Deadline "
                + deadline.getName()
                + " with "
                + Arrays.toString(Arrays.stream(otherCommands).map(Command::getName).toArray()));
  }

  private NamedCommands() {
    throw new UnsupportedOperationException("This is a utility class");
  }
}
