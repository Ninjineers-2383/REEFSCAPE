package frc.robot.subsystems.digital_sensor;

public class DigitalSensorConstants {
  public record DigitalSensorConfig(int id, boolean invert) {}

  public static final DigitalSensorConfig OUTTAKE_TOP_BREAK_CONFIG =
      new DigitalSensorConfig(1, true);
  public static final DigitalSensorConfig OUTTAKE_BOTTOM_BREAK_CONFIG =
      new DigitalSensorConfig(0, true);
}
