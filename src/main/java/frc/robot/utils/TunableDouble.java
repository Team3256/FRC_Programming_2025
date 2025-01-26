package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Date;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TunableDouble extends Tunable<Double> implements DoubleSupplier {
  private LoggedNetworkNumber loggedNetworkNumber;

  private TunableDouble(Double value, Date tunedDate, String author, String notes) {
    super(value, tunedDate, author, notes);
    loggedNetworkNumber = null;
  }

  public TunableDouble toLive(String key) {
    loggedNetworkNumber =
        value == null
            ? new LoggedNetworkNumber("/Tunable/" + key)
            : new LoggedNetworkNumber("/Tunable/" + key, value);
    return this;
  }

  public Double use() {
    if (loggedNetworkNumber != null) {
      return loggedNetworkNumber.get();
    }
    return super.use();
  }

  public Trigger onChange(Consumer<Double> consumer) {
    if (loggedNetworkNumber == null) {
      throw new IllegalStateException("onChange cannot be called on a non-live Tunable");
    }
    return new Trigger(() -> value != loggedNetworkNumber.get())
        .onTrue(
            Commands.runOnce(
                () -> {
                  value = loggedNetworkNumber.get();
                  consumer.accept(value);
                }));
  }

  public double getAsDouble() {
    return use();
  }
}
