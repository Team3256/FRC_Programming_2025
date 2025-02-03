// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.warriorb.lib.drivers;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;

public class StreamDeck implements Mapper.IMapped {
  private NetworkTable nt4Table;
  private Map<String, String> buttonMap = new HashMap<>();

  public StreamDeck(String tableKey, NetworkTableInstance ntInstance) {
    this.nt4Table = ntInstance.getTable(tableKey);
  }

  public StreamDeck(String tableKey) {
    this(tableKey, NetworkTableInstance.getDefault());
  }

  public StreamDeck() {
    this("StreamDeck");
  }

  public Trigger on(String key, String description) {
    BooleanSubscriber keySubscriber = nt4Table.getBooleanTopic(key).subscribe(false);
    buttonMap.put(key, description);
    return new Trigger(() -> keySubscriber.get());
  }

  public Trigger on(String key) {
    return this.on(key, key);
  }

  public String getName() {
    return "streamdeck";
  }

  public Map<String, String> getMap() {
    return buttonMap;
  }
}
