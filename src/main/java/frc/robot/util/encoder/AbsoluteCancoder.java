package frc.robot.util.encoder;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;

public class AbsoluteCancoder implements IAbsoluteEncoder {
  private final CANcoder encoder;

  public AbsoluteCancoder(int id, String canbus, CANcoderConfiguration config) {
    this.encoder = new CANcoder(id, canbus);

    tryUntilOk(5, () -> this.encoder.getConfigurator().apply(config));
  }

  @Override
  public Rotation2d getAbsoluteAngle() {
    return Rotation2d.fromRotations(this.encoder.getAbsolutePosition().getValueAsDouble());
  }

  @Override
  public boolean isConnected() {
    return encoder.isConnected();
  }
}
