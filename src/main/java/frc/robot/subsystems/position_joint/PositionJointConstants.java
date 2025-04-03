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
      new PositionJointGains(15.0, 5.0, 2.0, 0.3, 0.1, 3.2, 0.0, 25.0, 8.0, 0.0, 1.3, 0.04, 0.0);

  public static final PositionJointHardwareConfig ELEVATOR_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {10, 11},
          new boolean[] {false, false},
          (40.281 + 0.07) / Units.inchesToMeters(77 - 25),
          40,
          GravityType.CONSTANT,
          EncoderType.INTERNAL,
          -1,
          new Rotation2d(),
          "Drive");

  public static final PositionJointGains PIVOT_GAINS =
      new PositionJointGains(45.0, 1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 3, -0.5, 1, 0.1, -0.25);

  public static final PositionJointHardwareConfig PIVOT_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {17},
          new boolean[] {true},
          46 + (2.0 / 3.0),
          40,
          GravityType.CONSTANT,
          EncoderType.EXTERNAL_CANCODER_PRO,
          17,
          Rotation2d.fromRotations(-0.219727 - 0.25),
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

  public static final PositionJointGains INTAKE_PIVOT_GAINS =
      new PositionJointGains(25, 0, 0, 0.5, 0, 0, 0, 1.5, 5, -0.161, 0.354, 0.05, 0.354);

  public static final PositionJointHardwareConfig INTAKE_PIVOT_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {8},
          new boolean[] {true},
          1 / ((14.0 / 64.0) * (20.0 / 64.0) * (18.0 / 44.0)),
          40,
          GravityType.CONSTANT,
          EncoderType.INTERNAL,
          -1,
          Rotation2d.fromRotations(0.354),
          "Drive");
}
