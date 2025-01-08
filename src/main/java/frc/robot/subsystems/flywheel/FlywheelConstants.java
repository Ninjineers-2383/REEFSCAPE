package frc.robot.subsystems.flywheel;

public class FlywheelConstants {
  public record FlywheelGains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public record FlywheelHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, String canBus) {}

  public static final FlywheelHardwareConfig CLAW_CONFIG =
      new FlywheelHardwareConfig(new int[] {14}, new boolean[] {true}, 24.0 / 48.0, "Drive");

  public static final FlywheelGains CLAW_GAINS = new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0);

  public static final FlywheelHardwareConfig CLIMBER_INTAKE_CONFIG =
      new FlywheelHardwareConfig(new int[] {15}, new boolean[] {true}, 24.0 / 48.0, "Drive");

  public static final FlywheelGains CLIMBER_INTAKE_GAINS =
      new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0);

  public static final FlywheelHardwareConfig INTAKE_CONFIG =
      new FlywheelHardwareConfig(new int[] {16}, new boolean[] {true}, 24.0 / 48.0, "Drive");

  public static final FlywheelGains INTAKE_GAINS =
      new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0);
}
