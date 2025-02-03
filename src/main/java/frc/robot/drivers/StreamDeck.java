// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.drivers;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class StreamDeck {

  private NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  private NetworkTable nt4Table;

  private BooleanSubscriber l1;
  private BooleanSubscriber l2;
  private BooleanSubscriber l3;
  private BooleanSubscriber l4;
  private BooleanSubscriber A;
  private BooleanSubscriber B;
  private BooleanSubscriber C;
  private BooleanSubscriber D;
  private BooleanSubscriber E;
  private BooleanSubscriber F;
  private BooleanSubscriber G;
  private BooleanSubscriber H;
  private BooleanSubscriber I;
  private BooleanSubscriber J;
  private BooleanSubscriber K;
  private BooleanSubscriber L;



  public StreamDeck(String tableName) {
    nt4Instance = NetworkTableInstance.getDefault();
    nt4Table = nt4Instance.getTable(tableName);

    l1 = nt4Table.getBooleanTopic("L1").subscribe(false);
    l2 = nt4Table.getBooleanTopic("L2").subscribe(false);
    l3 = nt4Table.getBooleanTopic("L3").subscribe(false);
    l4 = nt4Table.getBooleanTopic("L4").subscribe(false);
    A = nt4Table.getBooleanTopic("A").subscribe(false);
    B = nt4Table.getBooleanTopic("B").subscribe(false);
    C = nt4Table.getBooleanTopic("C").subscribe(false);
    D = nt4Table.getBooleanTopic("D").subscribe(false);
    E = nt4Table.getBooleanTopic("E").subscribe(false);
    F = nt4Table.getBooleanTopic("F").subscribe(false);
    G = nt4Table.getBooleanTopic("G").subscribe(false);
    H = nt4Table.getBooleanTopic("H").subscribe(false);
    I = nt4Table.getBooleanTopic("I").subscribe(false);
    J = nt4Table.getBooleanTopic("J").subscribe(false);
    K = nt4Table.getBooleanTopic("K").subscribe(false);
    L = nt4Table.getBooleanTopic("L").subscribe(false);



  }

  public Trigger l1() {
    return new Trigger(() -> l1.get());
  }
  public Trigger l2() {
    return new Trigger(() -> l2.get());
  }
  public Trigger l3() {
    return new Trigger(() -> l3.get());
  }
  public Trigger l4() {
    return new Trigger(() -> l4.get());
  }
  public Trigger A() {
    return new Trigger(() -> A.get());
  }
  public Trigger B() {
    return new Trigger(() -> B.get());
  }
  public Trigger C() {
    return new Trigger(() -> C.get());
  }
  public Trigger D() {
    return new Trigger(() -> D.get());
  }
  public Trigger E() {
    return new Trigger(() -> E.get());
  }
  public Trigger F() {
    return new Trigger(() -> F.get());
  }
  public Trigger G() {
    return new Trigger(() -> G.get());
  }
  public Trigger H() {
    return new Trigger(() -> H.get());
  }
  public Trigger I() {
    return new Trigger(() -> I.get());
  }
  public Trigger J() {
    return new Trigger(() -> J.get());
  }
  public Trigger K() {
    return new Trigger(() -> K.get());
  }
  public Trigger L() {
    return new Trigger(() -> L.get());
  }


}
