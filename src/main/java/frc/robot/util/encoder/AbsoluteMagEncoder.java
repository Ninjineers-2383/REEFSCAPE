package frc.robot.util.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AbsoluteMagEncoder implements IAbsoluteEncoder {
  private final DutyCycleEncoder encoder;

  public AbsoluteMagEncoder(int port) {
    this.encoder = new DutyCycleEncoder(port);
  }

  @Override
  public Rotation2d getAbsoluteAngle() {
    return Rotation2d.fromRotations(this.encoder.get() - 0.25);
  }

  @Override
  public boolean isConnected() {
    return encoder.isConnected();
  }
}
