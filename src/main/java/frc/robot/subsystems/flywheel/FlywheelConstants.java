package frc.robot.subsystems.flywheel;

public class FlywheelConstants {
  public record FlywheelGains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public record FlywheelHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, String canBus) {}
}
