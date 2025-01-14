package frc.robot.subsystems.digital_sensor;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.digital_sensor.DigitalSensorConstants.DigitalSensorConfig;

public class DigitalSensorIODigitialInput implements DigitalSensorIO {
  private final String name;

  private final DigitalInput digitalSensor;

  private final boolean invert;

  public DigitalSensorIODigitialInput(String name, DigitalSensorConfig config) {
    this.name = name;

    digitalSensor = new DigitalInput(config.id());
    invert = config.invert();
  }

  @Override
  public void updateInputs(DigitalSensorIOInputs inputs) {
    inputs.sensorActive = digitalSensor.get() ^ invert; // XOR
  }

  @Override
  public String getName() {
    return name;
  }
}
