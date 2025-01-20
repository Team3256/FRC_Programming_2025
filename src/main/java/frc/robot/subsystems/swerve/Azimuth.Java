// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.utils.DebugCommand;
import frc.robot.utils.ratelimiter.AdaptiveSlewRateLimiter;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

public class Azimuth extends DebugCommand {
  private CommandSwerveDrivetrain swerveSubsystem;
  private final AdaptiveSlewRateLimiter yLimiter;
  private final AdaptiveSlewRateLimiter xLimiter;
  private final DoubleSupplier leftY;
  private final DoubleSupplier leftX;
  private final DoubleSupplier triggerInput;
  private DoubleSupplier setpointAngle;
  private PIDController azimuthController;
  
  private SwerveRequest.FieldCentric driveWithAzimuth =
  new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed)
      .withRotationalRate(MaxAngularRate);

  /** Driver control */
  public Azimuth(
      CommandSwerveDrivetrain swerve,
      AdaptiveSlewRateLimiter yLimiter,
      AdaptiveSlewRateLimiter xLimiter,
      DoubleSupplier leftY,
      DoubleSupplier leftX,
      DoubleSupplier triggerInput,
      DoubleSupplier setpointAngle) {
    
      this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);

    this.yLimiter = yLimiter;
    this.xLimiter = xLimiter;
    this.leftY = leftY;
    this.leftX = leftX;
    this.triggerInput = triggerInput;
    this.setpointAngle = setpointAngle;
    this.azimuthController = new PIDController(aziDrivekP, aziDrivekI, aziDrivekD);
    azimuthController.enableContinuousInput(-180, 180);
    azimuthController.setTolerance(5, 5);
  }

  @Override
  public void execute() {

    double rotationPIDOutput =
        azimuthController.calculate(
          swerveSubsystem.getPigeon2().getRotation2d().getDegrees(), setpointAngle.getAsDouble());

    double finalRotationPIDOutput = MathUtil.clamp(rotationPIDOutput, -1.5 * Math.PI, 1.5 * Math.PI);

    if (Constants.FeatureFlags.kSwerveAccelerationLimitingEnabled) {
      this.swerveSubsystem.applyRequest(
              () ->
                  driveWithAzimuth
                      .withVelocityX(xLimiter.calculate(-leftY.getAsDouble()) * MaxSpeed)
                      .withVelocityY(yLimiter.calculate(-leftX.getAsDouble()) * MaxSpeed)
                      .withRotationalRate(finalRotationPIDOutput));

    } else {
      this.swerveSubsystem.applyRequest(
              () ->
                  driveWithAzimuth
                      .withVelocityX(-leftY.getAsDouble() * MaxSpeed)
                      .withVelocityY(-leftX.getAsDouble() * MaxSpeed)
                      .withRotationalRate(finalRotationPIDOutput));
    }
  }

  @Override
  public boolean isFinished() {
    // return manualRotating.getAsBoolean() || azimuthController.atSetpoint();
    return (triggerInput.getAsDouble() != 0.0 || azimuthController.atSetpoint());
  }
}