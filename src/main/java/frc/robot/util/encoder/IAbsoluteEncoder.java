package frc.robot.util.encoder;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IAbsoluteEncoder {
  public default Rotation2d getAbsoluteAngle() {
    return new Rotation2d();
  }

  public default boolean isConnected() {
    return true;
  }
}
