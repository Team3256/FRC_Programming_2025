// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.Elevator;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  public static enum StructureState {
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

  public static enum Gamepiece {
    CORAL,
    ALGAE,
    EMPTY
  }

  private StructureState state = StructureState.IDLE;
  private StructureState prevState = StructureState.IDLE;

  private Map<StructureState, Trigger> stateTriggers = new HashMap<StructureState, Trigger>();

  private Timer stateTimer = new Timer();

  private Gamepiece[] endeffectorStates = {Gamepiece.EMPTY, Gamepiece.EMPTY};
  //  private ScoringLocations selectedLocation = null;
  private int selectedLevel = 0;

  private Elevator elevator;
  private Arm arm;
  // TODO: replace with beambreak
  private Trigger hasCoral =
      new Trigger(
          () -> endeffectorStates[0] == Gamepiece.CORAL || endeffectorStates[1] == Gamepiece.CORAL);
  private Trigger hasAlgae =
      new Trigger(
          () -> endeffectorStates[0] == Gamepiece.ALGAE || endeffectorStates[1] == Gamepiece.ALGAE);

  public Superstructure(Elevator elevator, Arm arm) {
    this.elevator = elevator;
    this.arm = arm;
    stateTimer.start();

    for (StructureState state : StructureState.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state));
    }
    configStateTransitions();
  }

  public void configStateTransitions() {
    // TODO: button bindings for set state to reef, outtake (or outtake and reef), etc
    // make controller shake if tried to transition to invalid state, but still do it anyway

    // XXX: dealgae may need diff states
    // which means we may need to select different height dep on selected scoring type
    // TODO: retract ground intakes

    stateTriggers.get(StructureState.REEF).onTrue(elevator.toReefLevel(selectedLevel));
    stateTriggers
        .get(StructureState.SOURCE)
        .onTrue(elevator.toSource().alongWith(arm.setPosition(ArmConstants.kSourceAngle)))
        .and(hasCoral)
        // TODO: update the endeffector states before setState
        .onTrue(setState(StructureState.PRESCORE));

    // TODO: extend ground intakes first
    // todo: a lot of stuff too
    stateTriggers.get(StructureState.GROUND).onTrue(elevator.toGround());
    // TODO: initiate rollers
    stateTriggers
        .get(StructureState.SCORE_CORAL)
        .onTrue(
            // TODO: + 180 depending on where the coral was inputted?
            arm.setPosition(
                selectedLevel == 0
                    ? ArmConstants.kTroughAngle
                    : selectedLevel == 1 || selectedLevel == 2
                        ? ArmConstants.kL2L3Angle
                        : ArmConstants.kL4Angle))
        .and(hasCoral.negate())
        .onTrue(setState(hasAlgae.getAsBoolean() ? StructureState.PRESCORE : StructureState.IDLE));
    // TODO: determine kAlgaeA or deAlgaeB....
    stateTriggers
        .get(StructureState.DEALGAE)
        .onTrue(arm.setPosition(ArmConstants.kAlgaeAAngle))
        .and(hasAlgae)
        // TODO: update the endeffector states before setState

        .onTrue(setState(StructureState.PRESCORE));
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
