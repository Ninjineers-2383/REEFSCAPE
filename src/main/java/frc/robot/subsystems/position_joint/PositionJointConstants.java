package frc.robot.subsystems.position_joint;

public class PositionJointConstants {
  public enum GravityType {
    CONSTANT,
    COSINE,
    SIN
  }

  public record PositionJointGains(
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

  // Position Joint Gear Ratio should be multiplied by Math.PI * 2 for rotation joints to convert
  // from rotations to radians
  public record PositionJointHardwareConfig(
      int[] canIds,
      boolean[] reversed,
      double gearRatio,
      double currentLimit,
      GravityType gravity,
      String canBus) {}

  public static final PositionJointGains pivot_gains =
      new PositionJointGains(1.5, 0.0, 0.0, 0.5, 1.0, 2.0, 0.0, 10.0, 20.0, 0.0, Math.PI, 0.2);

  public static final PositionJointHardwareConfig pivot_config =
      new PositionJointHardwareConfig(
          new int[] {10},
          new boolean[] {true},
          1 / 85.33333 * 2 * Math.PI,
          40,
          GravityType.SIN,
          "");
}
