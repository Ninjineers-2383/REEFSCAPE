package frc.robot.subsystems.drive.azimuth_motor;

public class AzimuthMotorIOReplay implements AzimuthMotorIO {
  private final String name;

  public AzimuthMotorIOReplay(String name) {
    this.name = name;
  }

  @Override
  public String getName() {
    return name;
  }
}
