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

  private ManipulatorSide manipulatorSide = ManipulatorSide.RIGHT;

  private StructureState state = StructureState.IDLE;
  private StructureState prevState = StructureState.IDLE;

  private Map<StructureState, Trigger> stateTriggers = new HashMap<StructureState, Trigger>();

  private Map<StructureState, Trigger> prevStateTriggers = new HashMap<StructureState, Trigger>();

  private final Trigger rightManipulatorSide =
      new Trigger(() -> this.manipulatorSide == ManipulatorSide.RIGHT);

  private Timer stateTimer = new Timer();

  private final Elevator elevator;
  private final EndEffector endEffector;
  private final Arm arm;

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
    stateTriggers.get(StructureState.IDLE).onTrue(endEffector.off());

    stateTriggers
        .get(StructureState.L1)
        .onTrue(elevator.toReefLevel(0))
        .onTrue(arm.toReefLevel(0, rightManipulatorSide));

    stateTriggers.get(StructureState.L2).onTrue(elevator.toReefLevel(1));
    stateTriggers.get(StructureState.L3).onTrue(elevator.toReefLevel(2));
    stateTriggers
        .get(StructureState.L4)
        .onTrue(elevator.toReefLevel(3))
        .onTrue(arm.toReefLevel(2, rightManipulatorSide));
    stateTriggers
        .get(StructureState.L2)
        .or(stateTriggers.get(StructureState.L3))
        .onTrue(arm.toReefLevel(1, rightManipulatorSide));

    stateTriggers
        .get(StructureState.SCORE_CORAL)
        .and(prevStateTriggers.get(StructureState.L1))
        .onTrue(endEffector.setL1Velocity(rightManipulatorSide));
    stateTriggers
        .get(StructureState.SCORE_CORAL)
        .and(prevStateTriggers.get(StructureState.L2).or(prevStateTriggers.get(StructureState.L3)))
        .onTrue(endEffector.setL2L3Velocity(rightManipulatorSide));
    stateTriggers
        .get(StructureState.SCORE_CORAL)
        .and(prevStateTriggers.get(StructureState.L4))
        .onTrue(endEffector.setL4Velocity(rightManipulatorSide));

    stateTriggers.get(StructureState.DEALGAE_L2).onTrue(elevator.toDealgaeLevel(0));

    stateTriggers.get(StructureState.DEALGAE_L3).onTrue(elevator.toDealgaeLevel(1));
    stateTriggers
        .get(StructureState.DEALGAE_L2)
        .or(stateTriggers.get(StructureState.DEALGAE_L3))
        .onTrue(arm.toDealgaeLevel(rightManipulatorSide));

    stateTriggers
        .get(StructureState.PRESOURCE)
        .onTrue(elevator.setPosition(ElevatorConstants.sourcePosition.in(Rotations)))
        .and(elevator.isSafeForArm)
        .onTrue(this.setState(StructureState.SOURCE));
    stateTriggers
        .get(StructureState.SOURCE)
        .onTrue(arm.toSourceLevel(rightManipulatorSide))
        .onTrue(endEffector.setSourceVelocity(rightManipulatorSide))
        .and(endEffector.beamBreak)
        .onTrue(this.setState(StructureState.PREHOME));

    stateTriggers
        .get(StructureState.PREHOME)
        .onTrue(endEffector.off())
        .onTrue(elevator.toArmSafePosition());
    stateTriggers
        .get(StructureState.PREHOME)
        .and(prevStateTriggers.get(StructureState.SOURCE))
        .and(elevator.isSafeForArm)
        .onTrue(arm.toHome(rightManipulatorSide.negate()))
        .and(arm.isSafePosition)
        .onTrue(this.setState(StructureState.HOME));
    stateTriggers
        .get(StructureState.PREHOME)
        .and(prevStateTriggers.get(StructureState.SOURCE).negate())
        .and(elevator.isSafeForArm)
        .onTrue(arm.toHome())
        .and(arm.isSafePosition)
        .onTrue(this.setState(StructureState.HOME));
    stateTriggers
        .get(StructureState.HOME)
        .and(prevStateTriggers.get(StructureState.PREHOME))
        .onTrue(elevator.toHome())
        .onTrue(endEffector.off())
        .and(arm.reachedPosition.and(elevator.reachedPosition))
        .onTrue(this.setState(StructureState.IDLE));
    //    stateTriggers
    //        .get(StructureState.HOME)
    //        .and(prevStateTriggers.get(StructureState.PREHOME).negate())
    //        .onTrue(this.setState(StructureState.PREHOME));
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
          this.prevState = this.state == state ? this.prevState : this.state;
          this.state = state;
          this.stateTimer.restart();
        });
  }

  public StructureState getState() {
    return this.state;
  }

  public StructureState getPrevState() {
    return this.prevState;
  }

  public Command setManipulatorSide(ManipulatorSide side) {
    return Commands.runOnce(
        () -> {
          this.manipulatorSide = side;
          // set manipulator side
        });
  }
}
