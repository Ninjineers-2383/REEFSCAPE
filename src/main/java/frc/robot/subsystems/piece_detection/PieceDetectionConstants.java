package frc.robot.subsystems.piece_detection;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class PieceDetectionConstants {
  public record PieceDetectionConfig(Transform3d robotToCameraTransform) {}

  public static final PieceDetectionConfig EXAMPLE_CONFIG =
      new PieceDetectionConfig(
          new Transform3d(new Translation3d(0.2, 0.1, 0), new Rotation3d(90, 20, 0)));
}
