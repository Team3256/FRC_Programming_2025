// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  public enum StructureState {
    IDLE,
    CLIMB,
    REEF,
    DEALGAE,
    PRESCORE,
    SOURCE,
    PROCESSOR,
    GROUND,
    SCORE_CORAL
  }

  public enum Gamepiece {
    CORAL,
    ALGAE,
    EMPTY
  }

  private StructureState state = StructureState.IDLE;
  private StructureState prevState = StructureState.IDLE;

  private final Map<StructureState, Trigger> stateTriggers = new HashMap<StructureState, Trigger>();

  private final Timer stateTimer = new Timer();

  private final Gamepiece[] endeffectorStates = {Gamepiece.EMPTY, Gamepiece.EMPTY};
  //  private ScoringLocations selectedLocation = null;
  private final int selectedLevel = 0;
  private Gamepiece selectedGamePiece = Gamepiece.EMPTY;
  private final Trigger hasCoral =
      new Trigger(
          () -> endeffectorStates[0] == Gamepiece.CORAL || endeffectorStates[1] == Gamepiece.CORAL);
  private final Trigger hasAlgae =
      new Trigger(
          () -> endeffectorStates[0] == Gamepiece.ALGAE || endeffectorStates[1] == Gamepiece.ALGAE);

  private final Elevator elevator;
  private final Arm arm;
  private final EndEffector endeffector;

  // TODO: replace with beambreak

  public Superstructure(
      Elevator elevator,
      Arm arm,
      EndEffector endeffector,
      boolean frontHasPreload,
      boolean backHasPreload) {
    this.elevator = elevator;
    this.arm = arm;
    this.endeffector = endeffector;
    if (frontHasPreload) {
      this.endeffectorStates[0] = Gamepiece.CORAL;
    }
    if (backHasPreload) {
      this.endeffectorStates[1] = Gamepiece.CORAL;
    }
    stateTimer.start();

    for (StructureState state : StructureState.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state));
    }
    this.endeffector
        .leftBeamBreak
        .onTrue(
            Commands.runOnce(
                () -> {
                  this.endeffectorStates[0] = selectedGamePiece;
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  this.endeffectorStates[0] = Gamepiece.EMPTY;
                }));
    this.endeffector
        .rightBeamBreak
        .onTrue(
            Commands.runOnce(
                () -> {
                  this.endeffectorStates[1] = selectedGamePiece;
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  this.endeffectorStates[1] = Gamepiece.EMPTY;
                }));
    configStateTransitions();
  }

  private Command prepareFor(Gamepiece gamepiece) {
    return Commands.runOnce(
        () -> {
          selectedGamePiece = gamepiece;
        });
  }

  // TODO: print warnings on invalid state transitions (or just verify and stop invalid
  // transitions)
  public void configStateTransitions() {
    // TODO: button bindings for set state to reef, outtake (or outtake and reef), etc
    // make controller shake if tried to transition to invalid state, but still do it anyway

    // XXX: dealgae may need diff states
    // which means we may need to select different height dep on selected scoring type
    // TODO: retract ground intakes afterwards
    stateTriggers.get(StructureState.REEF).onTrue(elevator.toReefLevel(selectedLevel));

    stateTriggers
        .get(StructureState.SOURCE)
        .onTrue(
            prepareFor(Gamepiece.CORAL)
                .andThen(elevator.toSource().alongWith(arm.setPosition(ArmConstants.kSourceAngle)))
                .alongWith(endeffector.intakeCoral()))
        // I could've also done .until(hasCoral).andThen(...)
        // chained on the command in the previous line
        // but 8033 does this style so...
        .and(hasCoral)
        .onTrue(setState(StructureState.PRESCORE));

    // TODO: extend ground intakes first
    // todo: a lot of stuff too
    stateTriggers.get(StructureState.GROUND).onTrue(elevator.toGround());
    stateTriggers
        .get(StructureState.SCORE_CORAL)
        // TODO: perhaps an override button?
        .and(hasCoral)
        .onTrue(
            arm.setPosition(
                    // XXX: is this correct?
                    Degrees.of(endeffectorStates[0] == Gamepiece.CORAL ? 0 : 90)
                        .plus(
                            selectedLevel == 0
                                ? ArmConstants.kTroughAngle
                                : selectedLevel == 1 || selectedLevel == 2
                                    ? ArmConstants.kL2L3Angle
                                    : ArmConstants.kL4Angle))
                .andThen(endeffector.outtakeCoral())
                .until(hasCoral.negate())
                .andThen(
                    setState(
                        hasAlgae.getAsBoolean() ? StructureState.PRESCORE : StructureState.IDLE)));
    // TODO: determine kAlgaeA or deAlgaeB....
    stateTriggers
        .get(StructureState.DEALGAE)
        // Verify valid state transition
        //            .and(stateTriggers
        //                    .get(StructureState.REEF))
        .onTrue(
            prepareFor(Gamepiece.ALGAE)
                .andThen(arm.setPosition(ArmConstants.kAlgaeAAngle))
                .alongWith(endeffector.intakeAlgae()))
        // See line 138
        .and(hasAlgae)
        .onTrue(setState(StructureState.PRESCORE));
    // TODO: processor
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
}
