package frc.robot.subsystems.flywheel;

import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelGains;
import frc.robot.util.MotorIO.MotorIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs extends MotorIOInputs {
    public double velocity = 0.0;

    public double desiredVelocity = 0.0;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setVelocity(double velocity) {}

  public default void setVoltage(double voltage) {}

  public default void setGains(FlywheelGains gains) {}

  public default String getName() {
    return "Flywheel";
  }
}
