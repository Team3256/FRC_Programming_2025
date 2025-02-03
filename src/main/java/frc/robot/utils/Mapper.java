package frc.robot.utils;

import com.google.gson.Gson;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Map;

public class Mapper {
  private Map<String, Map<String, String>> map = new HashMap<>();

  public Mapper() {}

  public interface IMapped {
    public String getName();

    public Map<String, String> getMap();
  }

  public void dumpControllerMap() {
    try {
      Files.write(
          Paths.get("./scripts/map.json"), getControllerMap().getBytes(StandardCharsets.UTF_8));
      System.out.println("File written successfully!");
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public String getControllerMap() {
    Gson gson = new Gson();
    return gson.toJson(map);
  }

  public void registerControllerMap(IMapped mapped) {
    map.put(mapped.getName(), mapped.getMap());
  }
}
