package frc.robot.subsystems.position_joint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

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
      double kTolerance,
      double kDefaultSetpoint) {}

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
      new PositionJointGains(20.0, 2.0, 0.0, 0.5, 0.3, 5.0, 0.0, 10.0, 3.0, 0.0, 1.3, 0.1, 0.0);

  public static final PositionJointHardwareConfig ELEVATOR_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {10, 11},
          new boolean[] {false, false},
          118.7 / Units.inchesToMeters(62.5 - 9.5),
          40,
          GravityType.CONSTANT,
          EncoderType.INTERNAL,
          -1,
          new Rotation2d(),
          "Drive");

  public static final PositionJointGains PIVOT_GAINS =
      new PositionJointGains(
          15.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.124,
          Rotation2d.fromDegrees(270).getRotations(),
          0.1,
          0.124);

  public static final PositionJointHardwareConfig PIVOT_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {17},
          new boolean[] {false},
          (14.16 - 4.95) / (1.0 / 4.0),
          40,
          GravityType.CONSTANT,
          EncoderType.INTERNAL,
          -1,
          Rotation2d.fromRotations(0.124),
          "Drive");

  public static final PositionJointGains CLIMBER_GAINS =
      new PositionJointGains(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0);

  public static final PositionJointHardwareConfig CLIMBER_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {13, 14},
          new boolean[] {true, false},
          1.0,
          40,
          GravityType.CONSTANT,
          EncoderType.INTERNAL,
          -1,
          new Rotation2d(),
          "Drive");
}
