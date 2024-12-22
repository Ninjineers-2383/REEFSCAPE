package frc.robot.subsystems.flywheel;

public class FlywheelConstants {
  public record FlywheelGains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public record FlywheelHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, String canBus) {}

  public static final FlywheelHardwareConfig EXAMPLE_CONFIG =
      new FlywheelHardwareConfig(new int[] {1}, new boolean[] {true}, 24.0 / 48.0, "CANBus");

  public static final FlywheelGains EXAMPLE_GAINS =
      new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0);
}
