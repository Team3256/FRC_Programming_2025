// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.utils.autoaim.CoralTargets;
import frc.robot.utils.autoaim.SourceIntakeTargets;

public class AutoRoutines {
  private final AutoFactory m_factory;

  private final AutoCommands m_autoCommands;

  private final Elevator m_elevator;
  private final CommandSwerveDrivetrain m_drivetrain;
  private final Arm m_arm;
  private final EndEffector m_endEffector;

  public AutoRoutines(
      AutoFactory factory,
      Elevator elevator,
      Arm arm,
      EndEffector endEffector,
      CommandSwerveDrivetrain drivetrain) {
    m_factory = factory;
    m_elevator = elevator;
    m_arm = arm;
    m_drivetrain = drivetrain;
    m_endEffector = endEffector;
    m_autoCommands = new AutoCommands(elevator, arm, endEffector);
  }

  public AutoRoutine mobilityLeft() {
    final AutoRoutine routine = m_factory.newRoutine("mobilityLeft");
    final AutoTrajectory mobilityTop = routine.trajectory("MobilityTop");
    routine.active().onTrue(mobilityTop.resetOdometry().andThen(mobilityTop.cmd()));
    return routine;
  }

  public AutoRoutine mobilityRight() {
    final AutoRoutine routine = m_factory.newRoutine("mobilityRight");
    final AutoTrajectory mobilityBottom = routine.trajectory("MobilityBottom");
    routine.active().onTrue(mobilityBottom.resetOdometry().andThen(mobilityBottom.cmd()));
    return routine;
  }

  public AutoRoutine l4PreloadH() {
    final AutoRoutine routine = m_factory.newRoutine("l4PreloadH");
    final AutoTrajectory preloadH = routine.trajectory("MID-H");

    routine
        .active()
        .onTrue(preloadH.resetOdometry().andThen(Commands.waitSeconds(5)).andThen(preloadH.cmd()));
    preloadH.atTimeBeforeEnd(.5).onTrue(m_autoCommands.goToL4());
    preloadH
        .done()
        .onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
                .andThen(m_autoCommands.scoreL4().asProxy())
                .until(m_endEffector.coralBeamBreak.negate().debounce(.5))
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () -> preloadH.getFinalPose().orElse(CoralTargets.BLUE_H.location)))
                .andThen(m_autoCommands.home().asProxy()));
    //    l4Preload.atTimeBeforeEnd(.5).onTrue(m_autoCommands.goToL4());
    //    l4Preload
    //        .done()
    //        .onTrue(
    //
    // Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
    //                .andThen(m_autoCommands.scoreL4())
    //                .andThen(
    //                    Commands.waitUntil(
    //                        m_endEffector
    //                            .leftBeamBreak
    //                            .negate()
    //                            .and(m_endEffector.rightBeamBreak.negate())))
    //                .andThen(m_autoCommands.home()));

    return routine;
  }

  public AutoRoutine l4PreloadG() {
    final AutoRoutine routine = m_factory.newRoutine("l4PreloadG");
    final AutoTrajectory preloadG = routine.trajectory("MID-G");

    routine
        .active()
        .onTrue(preloadG.resetOdometry().andThen(Commands.waitSeconds(5)).andThen(preloadG.cmd()));
    preloadG.atTimeBeforeEnd(.5).onTrue(m_autoCommands.goToL4());
    preloadG
        .done()
        .onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
                .andThen(m_autoCommands.scoreL4().asProxy())
                .until(m_endEffector.coralBeamBreak.negate().debounce(.5))
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () -> preloadG.getFinalPose().orElse(CoralTargets.BLUE_G.location)))
                .andThen(m_autoCommands.home().asProxy()));
    //    l4Preload.atTimeBeforeEnd(.5).onTrue(m_autoCommands.goToL4());
    //    l4Preload
    //        .done()
    //        .onTrue(
    //
    // Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
    //                .andThen(m_autoCommands.scoreL4())
    //                .andThen(
    //                    Commands.waitUntil(
    //                        m_endEffector
    //                            .leftBeamBreak
    //                            .negate()
    //                            .and(m_endEffector.rightBeamBreak.negate())))
    //                .andThen(m_autoCommands.home()));

    return routine;
  }

  public AutoRoutine test() {
    final AutoRoutine routine = m_factory.newRoutine("l4CenterPreloadRightSourceRight");
    final AutoTrajectory preloadH = routine.trajectory("MID-H");
    final AutoTrajectory HtoSource = routine.trajectory("H-SourceRight");
    final AutoTrajectory SourceToC = routine.trajectory("SourceRight-C");
    final AutoTrajectory CtoSource = routine.trajectory("C-SourceRight");
    final AutoTrajectory SourceToD = routine.trajectory("SourceRight-D");

    routine.active().onTrue(preloadH.resetOdometry().andThen(preloadH.cmd()));
    preloadH.done().onTrue(HtoSource.spawnCmd());
    HtoSource.done().onTrue(SourceToC.spawnCmd());
    SourceToC.done().onTrue(CtoSource.spawnCmd());
    CtoSource.done().onTrue(SourceToD.spawnCmd());
    SourceToD.done().onTrue(m_autoCommands.goToL4());

    return routine;
  }

  public AutoRoutine l4CenterPreloadRightSourceRight() {
    return genericL4RightPreloadRightSourceRight(
        "l4CenterPreloadRightSourceRight",
        "MID-G",
        "G-SourceRight",
        "SourceRight-C",
        "C-SourceRight",
        "SourceRight-D");
  }

  public AutoRoutine l4RightPreloadRightSourceRight() {
    return genericL4RightPreloadRightSourceRight(
        "l4RightPreloadRightSourceRight",
        "RIGHT-F",
        "F-SourceRight",
        "SourceRight-C",
        "C-SourceRight",
        "SourceRight-D");
  }

  public AutoRoutine l4LeftPreloadLeftSourceLeft() {
    return genericL4RightPreloadRightSourceRight(
        "l4LeftPreloadLeftSourceRight",
        "LEFT-I",
        "I-SourceLeft",
        "SourceLeft-K",
        "K-SourceLeft",
        "SourceLeft-L");
  }

  private AutoRoutine genericL4RightPreloadRightSourceRight(
      String name,
      String preloadTraj,
      String preloadToSourceTraj,
      String sourceToL41Traj,
      String l41ToSourceTraj,
      String sourceToL42Traj) {
    final AutoRoutine routine = m_factory.newRoutine(name);
    final AutoTrajectory preloadF = routine.trajectory(preloadTraj);
    final AutoTrajectory FtoSource = routine.trajectory(preloadToSourceTraj);
    final AutoTrajectory SourceToC = routine.trajectory(sourceToL41Traj);
    final AutoTrajectory CToSource = routine.trajectory(l41ToSourceTraj);
    final AutoTrajectory SourceToD = routine.trajectory(sourceToL42Traj);

    routine.active().onTrue(preloadF.resetOdometry().andThen(preloadF.cmd()));
    preloadF.atTimeBeforeEnd(.8).onTrue(m_autoCommands.goToL4());
    preloadF
        .done()
        .onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
                .andThen(m_autoCommands.scoreL4().asProxy())
                .until(m_endEffector.coralBeamBreak.negate())
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () -> preloadF.getFinalPose().orElse(CoralTargets.BLUE_F.location)))
                .andThen(
                    m_autoCommands
                        .goToSource()
                        .asProxy()
                        .alongWith(Commands.waitSeconds(.5).andThen(FtoSource.spawnCmd()))));

    FtoSource.atTimeBeforeEnd(.5)
        .onTrue(
            m_autoCommands
                .goToSource()
                .until(m_endEffector.coralBeamBreak)
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () ->
                            FtoSource.getFinalPose()
                                .orElse(SourceIntakeTargets.SOURCE_R_BLUE.location)))
                .andThen(m_autoCommands.home().alongWith(SourceToC.spawnCmd())));
    SourceToC.atTimeBeforeEnd(.8).onTrue(m_autoCommands.goToL4());
    SourceToC.done()
        .onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
                .andThen(m_autoCommands.scoreL4().asProxy())
                .until(m_endEffector.coralBeamBreak.negate())
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () -> SourceToC.getFinalPose().orElse(CoralTargets.BLUE_C.location)))
                .andThen(
                    m_autoCommands
                        .goToSource()
                        .asProxy()
                        .alongWith(Commands.waitSeconds(.5).andThen(CToSource.spawnCmd()))));

    CToSource.atTimeBeforeEnd(.5)
        .onTrue(
            m_autoCommands
                .goToSource()
                .until(m_endEffector.coralBeamBreak)
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () ->
                            CToSource.getFinalPose()
                                .orElse(SourceIntakeTargets.SOURCE_R_BLUE.location)))
                .andThen(m_autoCommands.home().alongWith(SourceToD.spawnCmd())));
    SourceToD.atTimeBeforeEnd(.8).onTrue(m_autoCommands.goToL4());
    SourceToD.done()
        .onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
                .andThen(m_autoCommands.scoreL4().asProxy())
                .until(m_endEffector.coralBeamBreak.negate())
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () -> SourceToD.getFinalPose().orElse(CoralTargets.BLUE_D.location)))
                .andThen(m_autoCommands.goToSource().asProxy()));

    return routine;
  }

  public AutoRoutine dealgae2LeftPreloadL4H() {
    final AutoRoutine routine = m_factory.newRoutine("dealgae2LeftPreloadL4H");
    final AutoTrajectory MidToG = routine.trajectory("MID-H");
    final AutoTrajectory GToGH = routine.trajectory("G-GH");
    final AutoTrajectory GHToBarge3 = routine.trajectory("GH-Barge3");
    final AutoTrajectory Barge3ToIJ = routine.trajectory("Barge3-IJ");
    final AutoTrajectory IJToBarge2 = routine.trajectory("IJ-Barge2");
    final AutoTrajectory Barge2ToH = routine.trajectory("Barge2-H");

    routine.active().onTrue(MidToG.resetOdometry().andThen(MidToG.cmd()));

    MidToG.atTimeBeforeEnd(.5).onTrue(m_autoCommands.goToL4());
    MidToG.done()
        .onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
                .andThen(m_autoCommands.scoreL4().asProxy())
                .until(m_endEffector.coralBeamBreak.negate())
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () -> MidToG.getFinalPose().orElse(CoralTargets.BLUE_H.location)))
                .andThen(
                    m_autoCommands
                        .home()
                        .asProxy()
                        .alongWith(Commands.waitSeconds(.2).andThen(GToGH.spawnCmd()))));
    GToGH.atTimeBeforeEnd(.7)
        .onTrue(m_autoCommands.goToL2Dealgae().alongWith(m_autoCommands.dealgaeify()));
    GToGH.done()
        .onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
                .andThen(m_autoCommands.dealgaeify().asProxy())
                .until(m_endEffector.algaeBeamBreak.debounce(.1))
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () -> GToGH.getFinalPose().orElse(CoralTargets.BLUE_G.location)))
                .andThen(
                    m_autoCommands
                        .homeDealgae()
                        .asProxy()
                        .alongWith(Commands.waitSeconds(.1).andThen(GHToBarge3.spawnCmd()))));
    GHToBarge3.atTimeBeforeEnd(.5).onTrue(m_autoCommands.goToBarge());
    GHToBarge3.done()
        .onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.15))
                .andThen(m_autoCommands.scoreBarge().asProxy())
                .until(m_endEffector.algaeBeamBreak.negate().debounce(.1))
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () ->
                            GHToBarge3.getFinalPose()
                                .orElse(
                                    new Pose2d(
                                        7.565075874328613,
                                        5.123231410980225,
                                        Rotation2d.fromRadians(4.71)))))
                .andThen(
                    m_autoCommands
                        .home()
                        .asProxy()
                        .alongWith(Commands.waitSeconds(.1).andThen(Barge3ToIJ.spawnCmd()))));
    Barge3ToIJ.atTimeBeforeEnd(1)
        .onTrue(m_autoCommands.goToL3Dealgae().alongWith(m_autoCommands.dealgaeify()));
    Barge3ToIJ.done()
        .onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.1))
                .andThen(m_autoCommands.dealgaeify().asProxy())
                .until(m_endEffector.algaeBeamBreak.debounce(.1))
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () -> Barge3ToIJ.getFinalPose().orElse(CoralTargets.BLUE_I.location)))
                .andThen(
                    m_autoCommands
                        .homeDealgae()
                        .asProxy()
                        .alongWith(Commands.waitSeconds(.5).andThen(IJToBarge2.spawnCmd()))));
    IJToBarge2.atTimeBeforeEnd(.7).onTrue(m_autoCommands.goToBarge());
    IJToBarge2.done()
        .onTrue(
            Commands.waitUntil(m_arm.reachedPosition.and(m_elevator.reachedPosition).debounce(.15))
                .andThen(m_autoCommands.scoreBarge().asProxy())
                .until(m_endEffector.algaeBeamBreak.negate().debounce(.1))
                .deadlineFor(
                    m_drivetrain.pidToPose(
                        () ->
                            IJToBarge2.getFinalPose()
                                .orElse(
                                    new Pose2d(
                                        7.565075874328613,
                                        6.251801490783691,
                                        Rotation2d.fromRadians(4.71)))))
                .andThen(
                    m_autoCommands
                        .home()
                        .asProxy()
                        .alongWith(Commands.waitSeconds(.5).andThen(Barge2ToH.spawnCmd()))));

    return routine;
  }

  private static class AutoCommands {

    private final Elevator m_elevator;
    private final Arm m_arm;
    private final EndEffector m_endEffector;

    public AutoCommands(Elevator elevator, Arm arm, EndEffector endEffector) {

      m_elevator = elevator;
      m_arm = arm;
      m_endEffector = endEffector;
    }

    public Command goToL4() {
      return m_elevator
          .toReefLevel(3)
          .alongWith(
              Commands.waitUntil(new Trigger(() -> m_elevator.getPosition() > 4))
                  .withTimeout(.5)
                  .andThen(m_arm.toReefLevel(2, () -> true)));
    }

    public Command goToL3Dealgae() {
      return m_elevator.toDealgaeLevel(1).alongWith(m_arm.toDealgaeLevel(1, () -> true));
    }

    public Command goToL2Dealgae() {
      return m_elevator.toDealgaeLevel(0).alongWith(m_arm.toDealgaeLevel(0, () -> true));
    }

    public Command goToBarge() {
      return m_elevator
          .toBargePosition()
          .alongWith(
              Commands.waitUntil(new Trigger(() -> m_elevator.getPosition() > 3.5))
                  .withTimeout(.3)
                  .andThen(m_arm.toBargeLevel(() -> false)));
    }

    public Command dealgaeify() {
      return m_endEffector.setAlgaeIntakeVelocity();
    }

    public Command scoreBarge() {
      return m_endEffector.setAlgaeOuttakeVoltage();
    }

    public Command scoreL4() {
      return m_endEffector.setL4Voltage(() -> true);
    }

    public Command goToSource() {
      return m_elevator
          .setPosition(ElevatorConstants.sourcePosition.in(Rotations))
          .alongWith(m_arm.toSourceLevel())
          .alongWith(m_endEffector.setSourceVelocity());
    }

    public Command home() {
      return m_elevator.toHome().alongWith(m_endEffector.coralOff()).alongWith(m_arm.toHome());
    }

    public Command homeDealgae() {
      return m_arm
          .toHome()
          .alongWith(Commands.waitUntil(m_arm.isSafePosition).andThen(m_elevator.toHome()))
          .alongWith(m_endEffector.coralOff());
    }
  }
}
