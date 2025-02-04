// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autogen.ScoringTypes;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.endeffector.EndEffector;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  public static enum StructureState {
    IDLE,
    L4,
    L3,
    L2,
    L1,
    DEALGAE_L3,
    DEALGAE_L2,
    PRESOURCE,
    SOURCE,
    Barge,
    CLIMB,
    PROCESSOR,
    SCORE_CORAL,
    PREHOME,
    HOME
  }

  public static enum ManipulatorSide {
    LEFT,
    RIGHT
  }

  private ManipulatorSide manipulatorSide = ManipulatorSide.LEFT;

  private StructureState state = StructureState.IDLE;
  private StructureState prevState = StructureState.IDLE;

  private Map<StructureState, Trigger> stateTriggers = new HashMap<StructureState, Trigger>();

  private Map<StructureState, Trigger> prevStateTriggers = new HashMap<StructureState, Trigger>();

  private final Trigger leftManipulatorSide =
      new Trigger(() -> this.manipulatorSide == ManipulatorSide.LEFT);

  private final Trigger rightManipulatorSide =
      new Trigger(() -> this.manipulatorSide == ManipulatorSide.RIGHT);

  private Timer stateTimer = new Timer();

  private final Elevator elevator;
  private final EndEffector endEffector;
  private final Arm arm;

  public static enum AutoCommands {
    CORAL,
    NONE,
    DONE
  }

  private AutoCommands currentAuto = AutoCommands.NONE;

  public Superstructure(Elevator elevator, EndEffector endEffector, Arm arm) {
    this.elevator = elevator;
    this.endEffector = endEffector;
    this.arm = arm;

    stateTimer.start();

    for (StructureState state : StructureState.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state));
    }
    for (StructureState state : StructureState.values()) {
      prevStateTriggers.put(state, new Trigger(() -> this.prevState == state));
    }

    configStateTransitions();
  }

  public void configStateTransitions() {
    stateTriggers.get(StructureState.IDLE);

    stateTriggers.get(StructureState.L1).onTrue(elevator.toReefLevel(0));
    stateTriggers.get(StructureState.L1).and(leftManipulatorSide).onTrue(arm.toLeftReefLevel(0));
    stateTriggers.get(StructureState.L1).and(rightManipulatorSide).onTrue(arm.toRightReefLevel(0));

    stateTriggers.get(StructureState.L2).onTrue(elevator.toReefLevel(1));
    stateTriggers.get(StructureState.L2).onTrue(elevator.toReefLevel(3));
    stateTriggers
        .get(StructureState.L2)
        .or(stateTriggers.get(StructureState.L3))
        .and(leftManipulatorSide)
        .onTrue(arm.toLeftReefLevel(1));
    stateTriggers
        .get(StructureState.L2)
        .or(stateTriggers.get(StructureState.L3))
        .and(rightManipulatorSide)
        .onTrue(arm.toRightReefLevel(1));

    stateTriggers
        .get(StructureState.SCORE_CORAL)
        .and(prevStateTriggers.get(StructureState.L1))
        .onTrue(endEffector.setL1Velocity());
    stateTriggers
        .get(StructureState.SCORE_CORAL)
        .and(prevStateTriggers.get(StructureState.L2).or(prevStateTriggers.get(StructureState.L3)))
        .onTrue(endEffector.setL2L3Velocity());
    stateTriggers
        .get(StructureState.SCORE_CORAL)
        .and(prevStateTriggers.get(StructureState.L4))
        .onTrue(endEffector.setL4Velocity());

    stateTriggers
        .get(StructureState.L1)
        .or(stateTriggers.get(StructureState.L2))
        .or(stateTriggers.get(StructureState.L3))
        .or(stateTriggers.get(StructureState.L4))
        .and(this.elevator.reachedPosition.and(this.arm.reachedPosition))
        .and(isRunningAutoSequence(AutoCommands.CORAL))
        .onTrue(this.setState(StructureState.SCORE_CORAL));
    stateTriggers
        .get(StructureState.SCORE_CORAL)
        .and(isRunningAutoSequence(AutoCommands.CORAL))
        .and(endEffector.reachedVelocity)
        // TODO: feature flag?
        .and(
            (this.manipulatorSide == ManipulatorSide.LEFT
                    ? endEffector.leftBeamBreak
                    : endEffector.rightBeamBreak)
                .negate())
        .onTrue(this.finishAutoSequence());

    stateTriggers.get(StructureState.DEALGAE_L2).onTrue(elevator.toDealgaeLevel(0));
    stateTriggers
        .get(StructureState.DEALGAE_L2)
        .or(stateTriggers.get(StructureState.DEALGAE_L3))
        .and(leftManipulatorSide)
        .onTrue(arm.toLeftDealgaeLevel(0));
    stateTriggers
        .get(StructureState.DEALGAE_L2)
        .or(stateTriggers.get(StructureState.DEALGAE_L3))
        .and(rightManipulatorSide)
        .onTrue(arm.toRightDealgaeLevel());

    stateTriggers.get(StructureState.DEALGAE_L3).onTrue(elevator.toDealgaeLevel(1));

    stateTriggers
        .get(StructureState.PRESOURCE)
        .onTrue(elevator.setPosition(ElevatorConstants.sourcePosition.in(Rotations)))
        .and(elevator.reachedPosition)
        .onTrue(this.setState(StructureState.SOURCE));
    stateTriggers
        .get(StructureState.SOURCE)
        .and(leftManipulatorSide)
        .onTrue(arm.toLeftSourceLevel());
    stateTriggers
        .get(StructureState.SOURCE)
        .and(rightManipulatorSide)
        .onTrue(arm.toRightSourceLevel());

    stateTriggers
        .get(StructureState.SOURCE)
        .onTrue(endEffector.setSourceVelocity())
        .and(endEffector.leftBeamBreak.or(endEffector.rightBeamBreak))
        .onTrue(this.setState(StructureState.PREHOME));

    stateTriggers
        .get(StructureState.PREHOME)
        .onTrue(arm.toHome())
        .and(arm.reachedPosition)
        .onTrue(this.setState(StructureState.HOME));
    stateTriggers
        .get(StructureState.HOME)
        .onTrue(arm.toHome())
        .onTrue(elevator.toHome())
        .onTrue(endEffector.off());
  }

  // call manually
  public void periodic() {
    Logger.recordOutput(this.getClass().getSimpleName() + "/State", this.state.toString());
    Logger.recordOutput(this.getClass().getSimpleName() + "/PrevState", this.prevState.toString());
    Logger.recordOutput(this.getClass().getSimpleName() + "/StateTime", this.stateTimer.get());
  }

  public Command setState(StructureState state) {
    return Commands.runOnce(
        () -> {
          this.prevState = this.state;
          this.state = state;
          this.stateTimer.restart();
        });
  }

  public Command setManipulatorSide(ManipulatorSide side) {
    return Commands.runOnce(
        () -> {
          this.manipulatorSide = side;
          // set manipulator side
        });
  }

  private Trigger isRunningAutoSequence(AutoCommands auto) {
    // TODO: optimize into getting from a hashmap of cached triggers?
    return new Trigger(() -> this.currentAuto == auto);
  }

  private Command runAutoSequence(AutoCommands auto, Command command) {
    return Commands.runOnce(() -> this.currentAuto = auto)
        .andThen(command)
        .until(() -> this.currentAuto == AutoCommands.DONE)
        .andThen(() -> this.currentAuto = AutoCommands.NONE);
  }

  private Command finishAutoSequence() {
    return Commands.runOnce(() -> this.currentAuto = AutoCommands.DONE);
  }

  public Command scoreCoral(ScoringTypes location, ManipulatorSide side) {
    return this.runAutoSequence(
        AutoCommands.CORAL,
        this.setManipulatorSide(side)
            .andThen(
                this.setState(
                    switch (location) {
                      case L1 -> StructureState.L1;
                      case L2 -> StructureState.L2;
                      case L3 -> StructureState.L3;
                      case L4 -> StructureState.L4;
                    })));
  }
}
