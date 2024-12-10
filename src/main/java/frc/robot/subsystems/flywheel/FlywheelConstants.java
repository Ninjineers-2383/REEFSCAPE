package frc.robot.subsystems.flywheel;

public class FlywheelConstants {
  public record FlywheelGains(
      String name, double kP, double kI, double kD, double kS, double kV, double kA) {}

  public record FlywheelHardwareConfig(int[] canIds, boolean[] reversed) {}
}
