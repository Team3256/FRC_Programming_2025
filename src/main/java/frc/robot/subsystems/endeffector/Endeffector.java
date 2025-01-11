// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Endeffector extends SubsystemBase {

  private final WristIO wristIO;
  private final WristIOInputsAutoLogged wristIOAutoLogged = new WristIOInputsAutoLogged();
  private final RollerIO rollerIO;
  private final RollerIOInputsAutoLogged rollerIOAutoLogged = new RollerIOInputsAutoLogged();
  private final SysIdRoutine m_sysIdRoutine;

  public Endeffector(WristIO wristIO, RollerIO rollerIO) {
    this.wristIO = wristIO;
    this.rollerIO = rollerIO;
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Seconds), // Use default ramp rate (1 V/s)
                Volts.of(6), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            EndeffectorConstants.kSysIDOnWrist
                ? new SysIdRoutine.Mechanism(
                    (volts) ->
                        wristIO
                            .getMotor()
                            .setControl(wristIO.getVoltageRequest().withOutput(volts.in(Volts))),
                    null,
                    this)
                : new SysIdRoutine.Mechanism(
                    (volts) ->
                        rollerIO
                            .getMotor()
                            .setControl(rollerIO.getVoltageRequest().withOutput(volts.in(Volts))),
                    null,
                    this));
  }

  @Override
  public void periodic() {
    super.periodic();
    wristIO.updateInputs(wristIOAutoLogged);
    rollerIO.updateInputs(rollerIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName() + "-wrist", wristIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName() + "-roller", rollerIOAutoLogged);
  }

  public Command setPosition(double position) {
    return this.run(
        () ->
            wristIO.setPosition(
                position * EndeffectorConstants.SimulationConstants.kWristGearRatio));
  }

  //   public Command setVoltage(double voltage) {
  //     return this.run(() -> motorIO.setVoltage(voltage)).finallyDo(motorIO::off);
  //   }

  public Command off() {
    return this.runOnce(wristIO::off).alongWith(this.runOnce(rollerIO::off));
  }

  public Command zero() {
    return this.runOnce(wristIO::zero).alongWith(this.runOnce(rollerIO::zero));
  }

  // Level must be between 0 to 3
  public Command outtakeCoral(int level) {
    return this.runOnce(() -> this.wristIO.setPosition(EndeffectorConstants.kBranchPosition[level]))
        .andThen(this.runOnce(() -> this.rollerIO.setVelocity(EndeffectorConstants.kCoralSpeed)));
  }

  // TODO: ground intake?
  public Command intakeCoral() {
    return this.runOnce(
        () -> this.rollerIO.setVelocity(EndeffectorConstants.kCoralSpeed.unaryMinus()));
  }

  public Command scoreProcessor() {
    return this.runOnce(() -> this.wristIO.setPosition(EndeffectorConstants.kProcessorPosition))
        .andThen(this.runOnce(() -> this.rollerIO.setVelocity(EndeffectorConstants.kAlgaeSpeed)));
  }

  public Command intakeAlgae(boolean l2l3) {
    Command setPosition =
        this.runOnce(
            () ->
                this.wristIO.setPosition(
                    l2l3
                        ? EndeffectorConstants.kAlgaeL2L3Position
                        : EndeffectorConstants.kAlgaeL3L4Position));

    return setPosition.andThen(
        this.runOnce(
            () -> this.rollerIO.setVelocity(EndeffectorConstants.kAlgaeSpeed.unaryMinus())));
  }

  public Command intakeAlgae() {
    // TODO: get if L2L3 or L3L4 from odometry
    return intakeAlgae(true);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
