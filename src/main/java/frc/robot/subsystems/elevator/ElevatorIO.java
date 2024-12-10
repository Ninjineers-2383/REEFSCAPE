package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorGains;
import frc.robot.util.MotorIO.MotorIOInputs;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs extends MotorIOInputs {
    public double position = 0.0;
    public double desiredPosition = 0.0;

    public double velocity = 0.0;
    public double desiredVelocity = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {
  }

  public default void setPosition(double position, double velocity) {
  }

  public default void setGains(ElevatorGains gains) {
  }

  public default void setVoltage(double voltage) {
  }
}
