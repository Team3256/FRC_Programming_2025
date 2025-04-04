// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

  private final CANdle candle; // No need for an IO subsystem

  public LED() {
    this.candle = new CANdle(0, "rio");
    CANdleConfiguration config =
        new CANdleConfiguration(); // CANdle is not Phoenix 5; cannot use method chaining
    config.stripType = CANdle.LEDStripType.RGB;
    config.brightnessScalar = 1;
    config.statusLedOffWhenActive = true;
    candle.configAllSettings(config);
  }

  private void _reset() {
    candle.clearAnimation(0);
    this.animate(IndicatorAnimation.Default);
  }

  private void _animate(IndicatorAnimation animation) {
    candle.clearAnimation(0);
    candle.animate(animation.getAnimation(), 0);
  }

  public Command animate(IndicatorAnimation animation) {
    System.out.println("LED Animation: " + animation);
    return this.run(
            () -> {
              this._animate(animation);
            })
        .finallyDo(
            () -> {
              this._reset();
            });
  }
}
