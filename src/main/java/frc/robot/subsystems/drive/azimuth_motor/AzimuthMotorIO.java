package frc.robot.subsystems.drive.azimuth_motor;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorConstants.AzimuthMotorGains;
import org.littletonrobotics.junction.AutoLog;

public interface AzimuthMotorIO {
  @AutoLog
  public static class AzimuthMotorIOInputs {
    public double outputPositionRotations = 0.0;
    public double rotorPositionRotations = 0.0;
    public double desiredPositionRotations = 0.0;

    public double velocityRotationsPerSecond = 0.0;
    public double desiredVelocityRotationsPerSecond = 0.0;

    public boolean[] motorsConnected = {false};
    public boolean encoderConnected = false;

    public double[] motorPositions = {0.0};
    public double[] motorVelocities = {0.0};
    public double[] motorAccelerations = {0.0};

    public double[] motorVoltages = {0.0};
    public double[] motorCurrents = {0.0};

    public double[] odometryTimestamps = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  public default void updateInputs(AzimuthMotorIOInputs inputs) {}

  public default void setPosition(double position, double velocity) {}

  public default void setVoltage(double voltage) {}

  public default void setGains(AzimuthMotorGains gains) {}

  public String getName();
}
