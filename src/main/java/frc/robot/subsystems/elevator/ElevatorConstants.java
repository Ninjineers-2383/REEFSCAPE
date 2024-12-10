package frc.robot.subsystems.elevator;

public class ElevatorConstants {
  public record ElevatorGains(
      String name,
      double kP,
      double kI,
      double kD,
      double kS,
      double kG,
      double kV,
      double kA,
      double kMaxVelo,
      double kMaxAccel,
      double kMinPosition,
      double kMaxPosition,
      double kTolerance) {}

  public record ElevatorHardwareConfig(
      int[] canIds,
      boolean[] reversed,
      double gearRatio,
      double currentLimit) {}
}
