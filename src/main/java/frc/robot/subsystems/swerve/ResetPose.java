package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class ResetPose extends InstantCommand {
    private CommandSwerveDrivetrain swerveSubsystem;

    public ResetPose(
            CommandSwerveDrivetrain swerve) {
        this.swerveSubsystem = swerve;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        this.swerveSubsystem.resetPose(swerveSubsystem.getCurrentPose());
    }
}
