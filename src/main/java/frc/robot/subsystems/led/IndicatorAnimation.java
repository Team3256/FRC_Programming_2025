// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

public enum IndicatorAnimation {
  Default(new LarsonAnimation(255, 0, 255, 1, 0.54, 64, LarsonAnimation.BounceMode.Center, 6)),
  AutoAlign(new RainbowAnimation()), // Or, alternatively:
  // AutoAlign(new StrobeAnimation(0, 0, 244, 255, 0.1, 64)),
  CoralIntaken(new StrobeAnimation(0, 255, 0, 255, 0.1, 64));

  private Animation curAnimation;

  private IndicatorAnimation(Animation animation) {
    this.curAnimation = animation;
  }

  public Animation getAnimation() {
    return curAnimation;
  }
}
