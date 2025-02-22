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
    EXTERNAL_CANCODER_PRO,
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
      new PositionJointGains(27.0, 3.0, 0.0, 0.5, 0.3, 4.0, 0.0, 25.0, 8.0, 0.0, 1.3, 0.04, 0.0);

  public static final PositionJointHardwareConfig ELEVATOR_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {10, 11},
          new boolean[] {true, false},
          (59.373046875 + 7.27001953125) / Units.inchesToMeters(61 - 9),
          40,
          GravityType.CONSTANT,
          EncoderType.INTERNAL,
          -1,
          new Rotation2d(),
          "Drive");

  public static final PositionJointGains PIVOT_GAINS =
      new PositionJointGains(45.0, 1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 3, -0.5, 1, 0.1, 0.25);

  public static final PositionJointHardwareConfig PIVOT_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {17},
          new boolean[] {true},
          46 + (2.0 / 3.0),
          40,
          GravityType.CONSTANT,
          EncoderType.EXTERNAL_CANCODER_PRO,
          17,
          Rotation2d.fromRotations(0.332764 + 0.25),
          "Drive");

  public static final PositionJointGains CLIMBER_GAINS =
      new PositionJointGains(3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 290, 0, 0.0);

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

  public static final PositionJointGains FUNNEL_PIVOT_GAINS =
      new PositionJointGains(1, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0.4, 0, 0.0);

  public static final PositionJointHardwareConfig FUNNEL_PIVOT_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {8},
          new boolean[] {true},
          1.0,
          40,
          GravityType.CONSTANT,
          EncoderType.INTERNAL,
          -1,
          new Rotation2d(),
          "");
}
