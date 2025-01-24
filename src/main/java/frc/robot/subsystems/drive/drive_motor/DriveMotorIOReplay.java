package frc.robot.subsystems.drive.drive_motor;

public class DriveMotorIOReplay implements DriveMotorIO {
  private final String name;

  public DriveMotorIOReplay(String name) {
    this.name = name;
  }

  @Override
  public String getName() {
    return name;
  }
}
