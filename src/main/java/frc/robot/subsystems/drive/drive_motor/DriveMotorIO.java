package frc.robot.subsystems.drive.drive_motor;

import frc.robot.subsystems.drive.drive_motor.DriveMotorConstants.DriveMotorGains;
import org.littletonrobotics.junction.AutoLog;

public interface DriveMotorIO {
  @AutoLog
  public static class DriveMotorIOInputs {
    public double velocityRotationsPerSecond = 0.0;
    public double desiredVelocityRotationsPerSecond = 0.0;

    public double positionRotations = 0.0;

    public boolean[] motorsConnected = {false};

    // In phoenix native units
    public double[] motorPositions = {0.0};
    public double[] motorVelocities = {0.0};
    public double[] motorAccelerations = {0.0};

    public double[] motorVoltages = {0.0};
    public double[] motorCurrents = {0.0};

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
  }

  public default void updateInputs(DriveMotorIOInputs inputs) {}

  public default void setVelocity(double velocity) {}

  public default void setVoltage(double voltage) {}

  public default void setGains(DriveMotorGains gains) {}

  public String getName();
}
