// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

public class RightStickAbsolute {
  /* Region IDs */
  public static final int ABRegionID = 1;
  public static final int CDRegionID = 2;
  public static final int EFRegionID = 3;
  public static final int GHRegionID = 4;
  public static final int IJRegionID = 5;
  public static final int KLRegionID = 6;

  /* Boundaries */
  public static final double xCoefficientLA = 0;
  public static final double yInterceptLA = 0;

  public static final double xCoefficientBC = 0;
  public static final double yInterceptBC = 0;

  /* Boundary DE is the x-axis */

  public static final double xCoefficientFG = 0;
  public static final double yInterceptFG = 0;

  public static final double xCoefficientHI = 0;
  public static final double yInterceptHI = 0;

  /* Boundary KJ is the x-axis */

  // run check every 0.1 seconds
  public int regionCheck(boolean freak, double xAxis, double yAxis) {
    if (yAxis < 0) {
      if (((((xAxis * xCoefficientLA) + yInterceptLA) < yAxis))
          && ((((xAxis * xCoefficientBC) + yInterceptBC) < yAxis))) {
        return 1;
        // AB
      }
      if (((((xAxis * xCoefficientBC) + yInterceptBC) > yAxis))) {
        return 2;
        // CD
      }
      if (((((xAxis * xCoefficientLA) + yInterceptLA) > yAxis))) {
        return 3;
        // KL
      }
    } else if (yAxis > 0) {
      if (((((xAxis * xCoefficientFG) + yInterceptFG) < yAxis))) {
        return 3;
        // EF
      }
      if (((((xAxis * xCoefficientFG) + yInterceptFG) > yAxis))
          && ((((xAxis * xCoefficientHI) + yInterceptHI) > yAxis))) {
        return 4;
        // GH
      }
      if (((((xAxis * xCoefficientHI) + yInterceptHI) < yAxis))) {
        return 5;
        // IJ
      }
    }
    return 4;
  }
}
