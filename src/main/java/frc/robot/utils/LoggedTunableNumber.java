// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import frc.robot.Constants;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class LoggedTunableNumber implements DoubleSupplier {
  private static final String tableKey = "/Tuning";

  private final String key;
  private final LoggedNetworkNumber dashboardNumber;
  private final double defaultValue;
  private static final Map<String, Double> values = new HashMap<>();
  private static final Map<String, LoggedNetworkNumber> networkNumbers = new HashMap<>();
  private static final Map<String, ArrayList<Consumer<Double>>> callbacks = new HashMap<>();

  /**
   * Create a new LoggedTunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this.dashboardNumber = new LoggedNetworkNumber(tableKey + "/" + dashboardKey, defaultValue);
    this.key = dashboardKey;
    this.defaultValue = defaultValue;
    networkNumbers.put(dashboardKey, dashboardNumber);
  }

  /**
   * Create a new LoggedTunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public LoggedTunableNumber(String dashboardKey) {
    this(dashboardKey, 0.0);
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  public double get() {
    return Constants.FeatureFlags.kTuningModeEnabled ? dashboardNumber.get() : defaultValue;
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @param fallbackValue The default value to return if the number is not in the dashboard.
   *     Different from using `.initDefault` and then `.get` because `.initDefault` is idempotent
   * @return The current value
   */
  public double getOrUse(double fallbackValue) {
    dashboardNumber.setDefault(fallbackValue);
    return Constants.FeatureFlags.kTuningModeEnabled ? dashboardNumber.get() : defaultValue;
  }

  /**
   * Runs action if any of the tunableNumbers have changed
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple *
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @param action Callback to run when any of the tunable numbers have changed. Access tunable
   *     numbers in order inputted in method
   */
  public void onChanged(Consumer<Double> action) {
    callbacks.getOrDefault(key, new ArrayList<>()).add(action);
  }

  public static void periodic() {
    for (String key : values.keySet()) {
      double previous = values.get(key);
      double value = networkNumbers.get(key).get();
      values.put(key, value);
      if (value == previous) {
        continue;
      }
      for (Consumer<Double> callback : callbacks.getOrDefault(key, new ArrayList<>())) {
        callback.accept(value);
      }
    }
  }

  /** Runs action if any of the tunableNumbers have changed */
  public void onChanged(Runnable action) {
    onChanged(values -> action.run());
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}
