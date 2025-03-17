package frc.robot.subsystems.piece_detection;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class PieceDetectionConstants {
  public record PieceDetectionConfig(Transform3d robotToCameraTransform) {}

  public static final PieceDetectionConfig EXAMPLE_CONFIG =
      new PieceDetectionConfig(
          new Transform3d(
              new Translation3d(0, 0, Units.inchesToMeters(31.5 + 3.8)),
              new Rotation3d(0, -Units.degreesToRadians(35), Math.PI)));
}
