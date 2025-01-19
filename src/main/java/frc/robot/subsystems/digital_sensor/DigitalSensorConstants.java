package frc.robot.subsystems.digital_sensor;

public class DigitalSensorConstants {
  public record DigitalSensorConfig(int id, boolean invert) {}

  public static final DigitalSensorConfig OUTTAKE_BREAK_CONFIG = new DigitalSensorConfig(0, false);
}
