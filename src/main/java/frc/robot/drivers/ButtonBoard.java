// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.drivers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.warriorb.lib.drivers.StreamDeck;

public class ButtonBoard extends StreamDeck {

  public Trigger l1() {
    return super.on("l1");
  }

  public Trigger l2() {
    return super.on("l2");
  }

  public Trigger l3() {
    return super.on("l3");
  }

  public Trigger l4() {
    return super.on("l4");
  }

  public Trigger A() {
    return super.on("A");
  }

  public Trigger B() {
    return super.on("B");
  }

  public Trigger C() {
    return super.on("C");
  }

  public Trigger D() {
    return super.on("D");
  }

  public Trigger E() {
    return super.on("E");
  }

  public Trigger F() {
    return super.on("F");
  }

  public Trigger G() {
    return super.on("G");
  }

  public Trigger H() {
    return super.on("H");
  }

  public Trigger I() {
    return super.on("I");
  }

  public Trigger J() {
    return super.on("J");
  }

  public Trigger K() {
    return super.on("K");
  }

  public Trigger L() {
    return super.on("L");
  }
}
