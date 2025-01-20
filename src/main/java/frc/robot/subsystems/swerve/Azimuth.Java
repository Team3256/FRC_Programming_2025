// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.commands;

import static frc.robot.Constants.stickDeadband;
import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.robot.subsystems.swerve.SwerveConstants.aziDrivekP;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

public class Azimuth extends DebugCommandBase {
  private SwerveDrive swerveSubsystem;
  private DoubleSupplier setpointAngle;
  private PIDController azimuthController;
  
  private SwerveRequest.FieldCentric driveWithAzimuth = 
  new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed)
      .withRotationalRate(MaxAngularRate);

  /** Driver control */
  public Azimuth(
      CommandSwerveDrivetrain swerve,
      DoubleSupplier leftY,
      DoubleSupplier leftX,
      DoubleSupplier triggerInput,
      DoubleSupplier setpointAngle) {
    
      this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);

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
          swerveSubsystem.getHeading().getDegrees(), setpointAngle.getAsDouble());

    rotationPIDOutput = MathUtil.clamp(rotationPIDOutput, -1.5 * Math.PI, 1.5 * Math.PI);

    if (FeatureFlags.kSwerveAccelerationLimitingEnabled) {
      this.swerveSubsystem.applyRequest(
              () ->
                  driveWithAzimuth
                      .withVelocityX(swerveVelXRateLimiter.calculate(-leftY) * MaxSpeed)
                      .withVelocityY(swerveVelYRateLimiter.calculate(-leftX) * MaxSpeed)
                      .withRotationalRate(rotationPIDOutput));

    } else {
      this.swerveSubsystem.applyRequest(
              () ->
                  driveWithAzimuth
                      .withVelocityX(-leftY * MaxSpeed)
                      .withVelocityY(-leftX * MaxSpeed)
                      .withRotationalRate(rotationPIDOutput));
    }
  }

  @Override
  public boolean isFinished() {
    // return manualRotating.getAsBoolean() || azimuthController.atSetpoint();
    return (triggerInput != 0 || azimuthController.atSetpoint)
  }
}