// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.elevator.ElevatorConstants;

public final class SimMechs {
  private Distance kRobotWidth = Inches.of(27);
  public final Mechanism2d mech =
      new Mechanism2d(
          kRobotWidth.in(Meters), ElevatorConstants.SimulationConstants.kMaxHeight.in(Meters));

  private final MechanismRoot2d elevatorRoot =
      mech.getRoot(
          "Elevator",
          kRobotWidth.div(2).in(Meters),
          ElevatorConstants.SimulationConstants.kStartingHeight.in(Meters));

  private final MechanismLigament2d elevatorViz =
      elevatorRoot.append(
          new MechanismLigament2d(
              "Elevator", ElevatorConstants.SimulationConstants.kStartingHeight.in(Meters), 90));

  private final MechanismLigament2d armViz =
      elevatorViz.append(
          new MechanismLigament2d(
              "Arm", Inches.of(21.5).in(Meters), 0.0, 5.0, new Color8Bit(Color.kGreen)));

  private static SimMechs instance = null;

  private SimMechs() {}

  public static SimMechs getInstance() {
    if (instance == null) {
      instance = new SimMechs();
    }
    return instance;
  }

  public void updateArm(Angle angle) {
    armViz.setAngle(angle.in(Degrees));
  }

  public void updateElevator(Distance height) {
    elevatorViz.setLength(height.in(Meters));
  }

  public void publishToNT() {
    SmartDashboard.putData("RobotSim", mech);
  }
}
