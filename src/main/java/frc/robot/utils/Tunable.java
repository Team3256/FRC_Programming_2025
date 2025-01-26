import edu.wpi.first.wpilibj.DriverStation;
import java.util.Date;

public class Tunable<T> {
  public static int STALE_DATE_THRESHOLD = 7 * 24 * 60 * 60 * 1000;
  protected T value;
  private Date tunedDate;
  private String author;
  private String note;

  protected Tunable(T value, Date tunedDate, String author, String note) {
    this.value = value;
    this.tunedDate = tunedDate;
    this.author = author;
    this.note = note;
  }

  public static <T> Tunable<T> todo() {
    return new Tunable<T>(null, null, null, "TODO");
  }

  public static <T> Tunable<T> of(T value) {
    return new Tunable<T>(value, null, null, null);
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

  public T use() {
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
}
