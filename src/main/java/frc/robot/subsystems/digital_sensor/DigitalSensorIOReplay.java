package frc.robot.subsystems.digital_sensor;

public class DigitalSensorIOReplay implements DigitalSensorIO {
  private final String name;

  public DigitalSensorIOReplay(String name) {
    this.name = name;
  }

  @Override
  public String getName() {
    return name;
  }
}
