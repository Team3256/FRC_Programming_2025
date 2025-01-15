// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

public class SimViz {
  private static SimViz instance = null;

  // See https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/mech2d-widget.html
  // The main mechanism object, the "window" or "canvas" if you will
  private Mechanism2d robotMechanisms = new Mechanism2d(3, 3);

  private MechanismRoot2d elevatorRoot = robotMechanisms.getRoot("Elevator", 2, 0);
  private MechanismLigament2d elevatorShaft =
      elevatorRoot.append(
          new MechanismLigament2d(
              "ElevatorShaft", ElevatorConstants.SimulationConstants.kMinHeight.in(Meters), 90));

  private SimViz() {
    SmartDashboard.putData("RobotMechanisms", robotMechanisms);
  }

  public static SimViz getInstance() {
    if (instance == null) {
      instance = new SimViz();
    }
    return instance;
  }

  public void updateElevatorPosition(Distance position) {
    elevatorShaft.setLength(
        ElevatorConstants.SimulationConstants.kStartingHeight.plus(position).in(Meters));
  }
}
