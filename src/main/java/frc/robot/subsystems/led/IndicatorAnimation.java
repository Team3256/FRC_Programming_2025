// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.LarsonAnimation;
// import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

// size calculation for LarsonAnimation:
// size = Math.floor((numLEDs) * (6/64))

public enum IndicatorAnimation {
  Default(new LarsonAnimation(0, 255, 255, 1, 1, 300, LarsonAnimation.BounceMode.Center, 40)),
  // Default(new RainbowAnimation(255, 10, 8 + 50)),
  AutoAlignRunning(new StrobeAnimation(255, 0, 255, 255, 0.05, 300)), // Or, alternatively:
  AutoAligned(new StrobeAnimation(0, 255, 0, 255, .2, 300)),
  CoralIntaken(new StrobeAnimation(0, 0, 255, 255, .2, 300)),
  AlgaeIntaken(new StrobeAnimation(255, 0, 0, 255, .2, 300)),
  Climb(new RainbowAnimation(255, 10, 300));

  private Animation curAnimation;

  private IndicatorAnimation(Animation animation) {
    this.curAnimation = animation;
  }

  public Animation getAnimation() {
    return curAnimation;
  }
}
