// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.Util;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Rotations;

public class Arm extends DisableSubsystem {

  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged armIOAutoLogged = new ArmIOInputsAutoLogged();


  public final Trigger reachedPosition = new Trigger(() -> isAtPosition());
  private Angle requestedPosition = Rotations.of(0.0);

  public Arm(boolean enabled, ArmIO armIO) {
    super(enabled);

    this.armIO = armIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    armIO.updateInputs(armIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), armIOAutoLogged);
  }

  public Command setPosition(Angle position) {
    return this.run(() -> {armIO.setPosition(position); requestedPosition = position;});
  }

  public Command setVoltage(Voltage voltage) {
    return this.run(() -> armIO.setVoltage(voltage));
  }

  public Command toRightReefLevel(int level) {
    return this.setPosition(ArmConstants.reefRightPositions[level]);
  }

  public Command toLeftReefLevel(int level) {
    return this.setPosition(ArmConstants.reefLeftPositions[level]);
  }

    public Command toRightDealgaeLevel() {
        return this.setPosition(ArmConstants.dealgaeRightPosition);
    }

    public Command toLeftDealgaeLevel(int level) {
        return this.setPosition(ArmConstants.dealgaeLeftPosition);
    }

    public Command toRightSourceLevel() {
        return this.setPosition(ArmConstants.sourceRightPositions);
    }

    public Command toLeftSourceLevel() {
        return this.setPosition(ArmConstants.sourceLeftPositions);
    }
    public boolean isAtPosition() {
        return Util.epsilonEquals(armIOAutoLogged.armEncoderAbsolutePosition, requestedPosition.in(Rotations), 0.01);
    }

    public Command toHome() {
        return this.setPosition(ArmConstants.homePosition);
    }

  public Command off() {

    return this.runOnce(armIO::off);
  }
}
