package frc.robot.subsystems.flywheel;

public class FlywheelConstants {
  public record FlywheelGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kTolerance) {}

  public record FlywheelHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, String canBus) {}

  public static final FlywheelHardwareConfig OUTTAKE_CONFIG =
      new FlywheelHardwareConfig(new int[] {14}, new boolean[] {true}, 24.0 / 48.0, "Drive");

  public static final FlywheelGains OUTTAKE_GAINS =
      new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0, 1.0);

  public static final FlywheelHardwareConfig CLIMBER_INTAKE_CONFIG =
      new FlywheelHardwareConfig(new int[] {15}, new boolean[] {true}, 24.0 / 48.0, "Drive");

  public static final FlywheelGains CLIMBER_INTAKE_GAINS =
      new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0, 1.0);
}
