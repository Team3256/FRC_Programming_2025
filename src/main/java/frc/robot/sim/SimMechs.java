// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.sim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public final class SimMechs {

  public final Mechanism2d mech = new Mechanism2d(5, 5);

  private final MechanismRoot2d elevatorRoot = mech.getRoot("Elevator", 3.5, 0.2);

  private final MechanismLigament2d elevatorViz =
      elevatorRoot.append(new MechanismLigament2d("Elevator", 2, 90));

  private final MechanismLigament2d armViz =
      elevatorViz.append(new MechanismLigament2d("Arm", 1, -90, 5.0, new Color8Bit(Color.kGreen)));

  private final MechanismLigament2d algaeEndEffectorViz =
      armViz.append(
          new MechanismLigament2d(
              "Algae End Effector Flywheel", 0.35, 90, 2.5, new Color8Bit(Color.kRed)));
  private final MechanismLigament2d coralEndEffectorViz =
      armViz.append(
          new MechanismLigament2d(
              "Coral End Effector Flywheel", .25, 0.0, 2.5, new Color8Bit(Color.kYellow)));

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
    elevatorViz.setLength(height.in(Meters));
  }

  public void publishToNT() {
    SmartDashboard.putData("RobotSim", mech);
  }

  public void updateEndEffector(Angle algae, Angle coral) {
    algaeEndEffectorViz.setAngle(algaeEndEffectorViz.getAngle() + algae.in(Degrees));
    coralEndEffectorViz.setAngle(coralEndEffectorViz.getAngle() + coral.in(Degrees));
  }
}
