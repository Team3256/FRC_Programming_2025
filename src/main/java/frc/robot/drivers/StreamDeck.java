package frc.robot.drivers;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import org.warriorb.lib.definitions.Mapper;

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
