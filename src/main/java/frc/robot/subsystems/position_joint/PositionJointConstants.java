package frc.robot.subsystems.position_joint;

import edu.wpi.first.math.geometry.Rotation2d;

public class PositionJointConstants {
  public enum GravityType {
    CONSTANT,
    COSINE,
    // Not supported by TalonFX
    SINE
  }

  public enum EncoderType {
    INTERNAL,
    EXTERNAL_CANCODER,
    EXTERNAL_DIO,
    EXTERNAL_SPARK
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
      EncoderType encoderType,
      int encoderID,
      Rotation2d encoderOffset,
      String canBus) {}

  public static final PositionJointGains ELEVATOR_GAINS =
      new PositionJointGains(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 20.0, 0.0, Math.PI, 0.2);

  public static final PositionJointHardwareConfig ELEVATOR_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {10, 11},
          new boolean[] {false, true},
          85.33333 * 2 * Math.PI,
          40,
          GravityType.CONSTANT,
          EncoderType.INTERNAL,
          -1,
          new Rotation2d(),
          "drive");

  public static final PositionJointGains PIVOT_GAINS =
      new PositionJointGains(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 20.0, 0.0, Math.PI, 0.2);

  public static final PositionJointHardwareConfig PIVOT_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {12},
          new boolean[] {false},
          83.3333,
          40,
          GravityType.COSINE,
          EncoderType.INTERNAL,
          -1,
          new Rotation2d(),
          "drive");
}
