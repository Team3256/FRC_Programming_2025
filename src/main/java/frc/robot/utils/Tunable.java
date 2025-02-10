// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import java.util.Date;
import java.util.List;

public class Tunable<T> {
  public static int STALE_DATE_THRESHOLD = 7 * 24 * 60 * 60 * 1000;
  protected T value;
  private Date tunedDate;
  private String author;
  private String note;
  private Constants.RobotType robotType = null;

  protected Tunable(
      T value, Date tunedDate, String author, String note, Constants.RobotType robotType) {
    this.value = value;
    this.tunedDate = tunedDate;
    this.author = author;
    this.note = note;
    this.robotType = robotType;
  }

  public static <T> Tunable<T> todo() {
    return new Tunable<T>(null, null, null, "TODO", null);
  }

  public static <T> Tunable<T> of(T value, Constants.RobotType robotType) {
    return new Tunable<T>(value, null, null, null, robotType);
  }

  public Constants.RobotType getRobotType() {
    // Null check unnecessary.
    return robotType;
  }

  public Tunable<T> fromCAD() {
    this.note = "Values are from CAD";
    return this;
  }

  public Tunable<T> on(Date date) {
    this.tunedDate = date;
    return this;
  }

  public Tunable<T> by(String author) {
    this.author = author;
    return this;
  }

  public Tunable<T> withNote(String note) {
    this.note = note;
    return this;
  }

  public T get() {
    if (value == null) {
      throw new IllegalStateException("Not tuned");
    }

    if (tunedDate != null && (new Date().getTime() - tunedDate.getTime()) > STALE_DATE_THRESHOLD) {
      StringBuilder message = new StringBuilder();
      message.append("Tunable is stale (last tuned on ");
      message.append(tunedDate.toString());
      message.append(").");

      if (author != null) {
        message.append(" Please contact ");
        message.append(author);
        message.append(".");
      }
      if (note != null) {
        message.append(" Note: ");
        message.append(note);
      }
      DriverStation.reportWarning(message.toString(), false);
    }

    if (author == null) {
      DriverStation.reportWarning("Tunable is missing author", false);
    }

    return value;
  }

  public static <T> T use(List<Tunable<T>> tunables) {
    for (Tunable<T> tunable : tunables) {
      if (tunable.getRobotType() != Util.getRobotType()) {
        continue;
      }
      return tunable.get();
    }
    throw new IllegalStateException("No tunable is tuned");
  }
}
