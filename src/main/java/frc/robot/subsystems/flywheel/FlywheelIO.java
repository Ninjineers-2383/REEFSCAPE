package frc.robot.subsystems.flywheel;

import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelGains;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public double velocity = 0.0;
    public double desiredVelocity = 0.0;

    public double position = 0.0;

    public boolean[] motorsConnected = {false};

    public double[] motorPositions = {0.0};
    public double[] motorVelocities = {0.0};
    public double[] motorAccelerations = {0.0};

    public double[] motorVoltages = {0.0};
    public double[] motorCurrents = {0.0};
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setVelocity(double velocity) {}

  public default void setVoltage(double voltage) {}

  public default void setGains(FlywheelGains gains) {}

  public String getName();
}
