package frc.robot.autogen;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.rollers.Roller;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

import java.util.ArrayList;

public class NodeManager {


    private final CommandSwerveDrivetrain drivetrain;
    private final Roller rollers;

    private final AutoFactory factory;

    public NodeManager(CommandSwerveDrivetrain drivetrain, Roller rollers, AutoFactory factory) {
        this.drivetrain = drivetrain;
        this.rollers = rollers;
        this.factory = factory;
    }


    public AutoRoutine createAuto(PreloadNode preload, ArrayList<Node> nodes) {
        AutoRoutine routine = factory.newRoutine("Auto");

        AutoTrajectory preloadTraj = routine.trajectory(preload.startLocation().name()+"-"+preload.scoringLocation().name());
        routine.active().onTrue(preloadTraj.resetOdometry().andThen(preloadTraj.cmd()));
        rollers.setRollerVoltage(3);
        Trigger nextTrajTrigger = preloadTraj.done().debounce(3);
        ScoringLocations lastScoringLocation = preload.scoringLocation();
        for (Node node : nodes) {
            AutoTrajectory intakeTraj = routine.trajectory(lastScoringLocation.name()+"-"+node.intakeLocation().name());
            nextTrajTrigger.onTrue(intakeTraj.cmd());
            intakeTraj.done().onTrue(rollers.setRollerVoltage(3));
            AutoTrajectory scoringTraj = routine.trajectory(node.intakeLocation().name()+"-"+node.scoringLocation().name());
            intakeTraj.done().debounce(2).onTrue(scoringTraj.cmd());
            scoringTraj.done().onTrue(rollers.setRollerVoltage(-3));
            lastScoringLocation = node.scoringLocation();
            nextTrajTrigger = scoringTraj.done().debounce(2);
        }
        return routine;
    }
}
