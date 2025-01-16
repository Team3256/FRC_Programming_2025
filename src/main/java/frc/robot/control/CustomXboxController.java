// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.control;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class CustomXboxController extends XboxController {
  private static final double PRESS_THRESHOLD = 0.05;
  private double DEAD_BAND = 0.15;
  private boolean rumbling = false;
  public ButtonCheck aButton,
      bButton,
      xButton,
      yButton,
      startButton,
      backButton,
      leftBumper,
      rightBumper,
      leftCenterClick,
      rightCenterClick,
      leftTrigger,
      rightTrigger,
      POV0,
      POV90,
      POV180,
      POV270,
      POV135,
      POV225;
  public static final int A_BUTTON = 1;
  public static final int B_BUTTON = 2;
  public static final int X_BUTTON = 3;
  public static final int Y_BUTTON = 4;
  public static final int LEFT_BUMPER = 5;
  public static final int RIGHT_BUMPER = 6;
  public static final int BACK_BUTTON = 7;
  public static final int START_BUTTON = 8;
  public static final int LEFT_CENTER_CLICK = 9;
  public static final int RIGHT_CENTER_CLICK = 10;
  public static final int LEFT_TRIGGER = -2;
  public static final int RIGHT_TRIGGER = -3;
  public static final int POV_0 = -4;
  public static final int POV_90 = -5;
  public static final int POV_180 = -6;
  public static final int POV_270 = -7;
  public static final int POV_135 = -8;
  public static final int POV_225 = -9;

  public void setDeadband(double deadband) {
    DEAD_BAND = deadband;
  }

  public CustomXboxController(int usb) {
    super(usb);
    aButton = new ButtonCheck(A_BUTTON);
    bButton = new ButtonCheck(B_BUTTON);
    xButton = new ButtonCheck(X_BUTTON);
    yButton = new ButtonCheck(Y_BUTTON);
    startButton = new ButtonCheck(START_BUTTON);
    backButton = new ButtonCheck(BACK_BUTTON);
    leftBumper = new ButtonCheck(LEFT_BUMPER);
    rightBumper = new ButtonCheck(RIGHT_BUMPER);
    leftCenterClick = new ButtonCheck(LEFT_CENTER_CLICK);
    rightCenterClick = new ButtonCheck(RIGHT_CENTER_CLICK);
    leftTrigger = new ButtonCheck(LEFT_TRIGGER);
    rightTrigger = new ButtonCheck(RIGHT_TRIGGER);
    POV0 = new ButtonCheck(POV_0);
    POV90 = new ButtonCheck(POV_90);
    POV180 = new ButtonCheck(POV_180);
    POV270 = new ButtonCheck(POV_270);
    POV135 = new ButtonCheck(POV_135);
    POV225 = new ButtonCheck(POV_225);
  }

  @Override
  public double getLeftX() {
    return (Math.abs(getRawAxis(0)) > Math.abs(DEAD_BAND)) ? getRawAxis(0) : 0.0;
  }

  @Override
  public double getRightX() {
    return (Math.abs(getRawAxis(4)) > Math.abs(DEAD_BAND)) ? getRawAxis(4) : 0.0;
  }

  @Override
  public double getLeftY() {
    return (Math.abs(getRawAxis(1)) > Math.abs(DEAD_BAND)) ? getRawAxis(1) : 0.0;
  }

  @Override
  public double getRightY() {
    return (Math.abs(getRawAxis(5)) > Math.abs(DEAD_BAND)) ? getRawAxis(5) : 0.0;
  }

  @Override
  public double getLeftTriggerAxis() {
    return (Math.abs(getRawAxis(2)) > Math.abs(PRESS_THRESHOLD)) ? getRawAxis(2) : 0.0;
  }

  @Override
  public double getRightTriggerAxis() {
    return (Math.abs(getRawAxis(3)) > Math.abs(PRESS_THRESHOLD)) ? getRawAxis(3) : 0.0;
  }

  public double getTriggerAxes() {
    if (this.getLeftTriggerAxis() != 0 && this.getRightTriggerAxis() == 0) {
      return getLeftTriggerAxis();
    }
    if (this.getRightTriggerAxis() != 0 && this.getLeftTriggerAxis() == 0) {
      return getRightTriggerAxis();
    } else if (this.getLeftTriggerAxis() != 0 && this.getRightTriggerAxis() != 0) {
      return 0.0;
    } else {
      return 0.0;
    }
  }

  public Rotation2d getPOVDirection() {
    System.out.println(getPOV());
    return Rotation2d.fromDegrees(getPOV());
  }

  public void rumble(double rumblesPerSecond, double numberOfSeconds) {
    if (!rumbling) {
      RumbleThread r = new RumbleThread(rumblesPerSecond, numberOfSeconds);
      r.start();
    }
  }

  public boolean isRumbling() {
    return rumbling;
  }

  public class RumbleThread extends Thread {
    public double rumblesPerSec = 1;
    public long interval = 500;
    public double seconds = 1;
    public double startTime = 0;

    public RumbleThread(double rumblesPerSecond, double numberOfSeconds) {
      rumblesPerSec = rumblesPerSecond;
      seconds = numberOfSeconds;
      interval = (long) (1 / (rumblesPerSec * 2) * 1000);
    }

    public void run() {
      rumbling = true;
      startTime = Timer.getFPGATimestamp();
      try {
        while ((Timer.getFPGATimestamp() - startTime) < seconds) {
          setRumble(RumbleType.kLeftRumble, 1);
          setRumble(RumbleType.kRightRumble, 1);
          sleep(interval);
          setRumble(RumbleType.kLeftRumble, 0);
          setRumble(RumbleType.kRightRumble, 0);
          sleep(interval);
        }
      } catch (InterruptedException e) {
        rumbling = false;
        e.printStackTrace();
      }
      rumbling = false;
    }
  }

  public class ButtonCheck {
    boolean buttonCheck = false;
    boolean buttonActive = false;
    boolean activationReported = false;
    boolean longPressed = false;
    boolean longPressActivated = false;
    boolean hasBeenPressed = false;
    boolean longReleased = false;
    private double buttonStartTime = 0;
    private double longPressDuration = 0.3;

    public void setLongPressDuration(double seconds) {
      longPressDuration = seconds;
    }

    private int buttonNumber;

    public ButtonCheck(int id) {
      buttonNumber = id;
    }

    public void update() {
      if (buttonNumber > 0) {
        buttonCheck = getRawButton(buttonNumber);
      } else {
        switch (buttonNumber) {
          case LEFT_TRIGGER:
            buttonCheck = getLeftTriggerAxis() > 0;
            break;
          case RIGHT_TRIGGER:
            buttonCheck = getRightTriggerAxis() > 0;
            break;
          case POV_0:
            buttonCheck = (getPOV() == 0);
            break;
          case POV_90:
            buttonCheck = (getPOV() == 90);
            break;
          case POV_180:
            buttonCheck = (getPOV() == 180);
            break;
          case POV_270:
            buttonCheck = (getPOV() == 270);
            break;
          case POV_135:
            buttonCheck = (getPOV() == 135);
            break;
          case POV_225:
            buttonCheck = (getPOV() == 225);
            break;
          default:
            buttonCheck = false;
            break;
        }
      }
      if (buttonCheck) {
        if (buttonActive) {
          if (((Timer.getFPGATimestamp() - buttonStartTime) > longPressDuration)
              && !longPressActivated) {
            longPressActivated = true;
            longPressed = true;
            longReleased = false;
          }
        } else {
          buttonActive = true;
          activationReported = false;
          buttonStartTime = Timer.getFPGATimestamp();
        }
      } else {
        if (buttonActive) {
          buttonActive = false;
          activationReported = true;
          if (longPressActivated) {
            hasBeenPressed = false;
            longPressActivated = false;
            longPressed = false;
            longReleased = true;
          } else {
            hasBeenPressed = true;
          }
        }
      }
    }

    public static double deadBand(double val, double deadband) {
      return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

    /**
     * Returns true once the button is pressed, regardless of the activation duration. Only returns
     * true one time per button press, and is reset upon release.
     */
    public boolean wasActivated() {
      if (buttonActive && !activationReported) {
        activationReported = true;
        return true;
      }
      return false;
    }

    /**
     * Returns true once the button is released after being held for 0.5 seconds or less. Only
     * returns true one time per button press.
     */
    public boolean shortReleased() {
      if (hasBeenPressed) {
        hasBeenPressed = false;
        return true;
      }
      return false;
    }

    /**
     * Returns true once if the button is pressed for more than 0.5 seconds. Only true while the
     * button is still depressed; it becomes false once the button is released.
     */
    public boolean longPressed() {
      if (longPressed) {
        longPressed = false;
        return true;
      }
      return false;
    }

    /**
     * Returns true one time once the button is released after being held for more than 0.5 seconds.
     */
    public boolean longReleased() {
      if (longReleased) {
        longReleased = false;
        return true;
      }
      return false;
    }

    /** Returns true once the button is released, regardless of activation duration. */
    public boolean wasReleased() {
      return shortReleased() || longReleased();
    }

    /** Returns true if the button is currently being pressed. */
    public boolean isBeingPressed() {
      return buttonActive;
    }
  }

  public void update() {
    aButton.update();
    bButton.update();
    xButton.update();
    yButton.update();
    startButton.update();
    backButton.update();
    leftBumper.update();
    rightBumper.update();
    leftCenterClick.update();
    rightCenterClick.update();
    leftTrigger.update();
    rightTrigger.update();
    POV0.update();
    POV90.update();
    POV180.update();
    POV270.update();
  }
}
