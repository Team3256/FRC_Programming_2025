// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.sim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;

public final class SimMechs {

  public final Mechanism2d mech =
      new Mechanism2d(Constants.SimulationConstants.kDrivebaseWidth.in(Meters), 1);

  private final MechanismRoot2d elevatorRoot =
      mech.getRoot(
          "Elevator",
          Constants.SimulationConstants.kDrivebaseWidth.in(Meters) / 2,
          ElevatorConstants.SimulationConstants.kStartingHeight.in(Meters));

  private final MechanismLigament2d elevatorViz =
      elevatorRoot.append(
          new MechanismLigament2d(
              "Elevator",
              // Show a little stubby lol
              ElevatorConstants.SimulationConstants.kStartingHeight.plus(Inches.of(6)).in(Meters),
              90));

  private final MechanismLigament2d armViz =
      elevatorViz.append(
          new MechanismLigament2d(
              "Arm",
              ArmConstants.Sim.armLength.in(Meters) / 4,
              0.0,
              7,
              new Color8Bit(Color.kGreen)));

  private final MechanismLigament2d algaeEndEffectorViz =
      armViz.append(
          new MechanismLigament2d(
              "Algae End Effector Flywheel", .1, 90, 2.5, new Color8Bit(Color.kRed)));
  private final MechanismLigament2d coralEndEffectorViz =
      armViz.append(
          new MechanismLigament2d(
              "Coral End Effector Flywheel", .1, 0.0, 2.5, new Color8Bit(Color.kYellow)));

  private final MechanismRoot2d climbRoot =
      mech.getRoot("Climb", Constants.SimulationConstants.kDrivebaseWidth.in(Meters) / 2, 1);

  private final MechanismLigament2d climbViz =
      climbRoot.append(
          new MechanismLigament2d("Climb", ClimbConstants.sim.climbLength.in(Meters), 180));
  private static SimMechs instance = null;

  private SimMechs() {}

  public static SimMechs getInstance() {
    if (instance == null) {
      instance = new SimMechs();
    }
    return instance;
  }

  public void updateArm(Angle angle) {
    armViz.setAngle(angle.minus(Degrees.of(90)).in(Degrees));
  }

  public void updateElevator(Distance height) {
    elevatorViz.setLength(height.in(Meters) / 4);
  }

  public void updateClimb(Angle angle) {
    climbViz.setAngle(angle.in(Degrees));
  }

  public void publishToNT() {
    SmartDashboard.putData("RobotSim", mech);
  }

  public void updateEndEffector(Angle algae, Angle coral) {
    algaeEndEffectorViz.setAngle(algaeEndEffectorViz.getAngle() + algae.in(Degrees));
    coralEndEffectorViz.setAngle(coralEndEffectorViz.getAngle() + coral.in(Degrees));
  }
}
