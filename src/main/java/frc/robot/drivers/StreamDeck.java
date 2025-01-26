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

  private BooleanSubscriber streamDeckKey1Subscriber;

  public StreamDeck(String tableName) {
    nt4Instance = NetworkTableInstance.getDefault();
    nt4Table = nt4Instance.getTable(tableName);

    streamDeckKey1Subscriber = nt4Table.getBooleanTopic("key1").subscribe(false);
  }

  public Trigger key1() {
    return new Trigger(() -> streamDeckKey1Subscriber.get());
  }
}
