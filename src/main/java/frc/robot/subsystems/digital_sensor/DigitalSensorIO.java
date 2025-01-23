package frc.robot.subsystems.digital_sensor;

import org.littletonrobotics.junction.AutoLog;

public interface DigitalSensorIO {
  @AutoLog
  public class DigitalSensorIOInputs {
    public boolean sensorActive = false;
  }

  public default void updateInputs(DigitalSensorIOInputs inputs) {}

  public String getName();
}
