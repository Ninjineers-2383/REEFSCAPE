package frc.robot.subsystems.position_joint;

import frc.robot.subsystems.position_joint.PositionJointConstants.PositionJointGains;
import org.littletonrobotics.junction.AutoLog;

public interface PositionJointIO {
  @AutoLog
  public static class PositionJointIOInputs {
    public double outputPosition = 0.0;
    public double rotorPosition = 0.0;
    public double desiredPosition = 0.0;

    public double velocity = 0.0;
    public double desiredVelocity = 0.0;

    public boolean[] motorsConnected = {false};
    public boolean encoderConnected = false;

    public double[] motorPositions = {0.0};
    public double[] motorVelocities = {0.0};
    public double[] motorAccelerations = {0.0};

    public double[] motorVoltages = {0.0};
    public double[] motorCurrents = {0.0};
  }

  public default void updateInputs(PositionJointIOInputs inputs) {}

  public default void setPosition(double position, double velocity) {}

  public default void setVoltage(double voltage) {}

  public default void setGains(PositionJointGains gains) {}

  public String getName();
}
