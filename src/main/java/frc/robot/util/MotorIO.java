package frc.robot.util;

public interface MotorIO {
  public static class MotorIOInputs {
    public boolean[] motorsConnected = {false};

    public double[] motorPositions = {0.0};
    public double[] motorVelocities = {0.0};
    public double[] motorAccelerations = {0.0};

    public double[] motorVoltages = {0.0};
    public double[] motorCurrents = {0.0};
  }
}
