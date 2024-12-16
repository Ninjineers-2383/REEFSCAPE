package frc.robot.subsystems.flywheel;

public class FlywheelConstants {
  public record FlywheelGains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public record FlywheelHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, String canBus) {}

  public static final FlywheelHardwareConfig left_shooter =
      new FlywheelHardwareConfig(new int[] {1}, new boolean[] {true}, 24.0 / 48.0, "Drive");

  public static final FlywheelHardwareConfig right_shooter =
      new FlywheelHardwareConfig(new int[] {2}, new boolean[] {false}, 24.0 / 24.0, "Drive");

  public static final FlywheelGains left_shooter_gains =
      new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0);

  public static final FlywheelGains right_shooter_gains =
      new FlywheelGains(0.25, 0.0, 0.0, 0.0, 0.127, 0.0);

  public static final FlywheelHardwareConfig intake =
      new FlywheelHardwareConfig(
          new int[] {
            5,
          },
          new boolean[] {false},
          1.0,
          "Drive");

  public static final FlywheelGains intake_gains = new FlywheelGains(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}
