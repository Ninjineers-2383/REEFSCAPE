package frc.robot.subsystems.flywheel;

public class FlywheelIOReplay implements FlywheelIO {
  private final String name;

  public FlywheelIOReplay(String name) {
    this.name = name;
  }

  public String getName() {
    return name;
  }
}
