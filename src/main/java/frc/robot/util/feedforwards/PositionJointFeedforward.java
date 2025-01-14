package frc.robot.util.feedforwards;

public interface PositionJointFeedforward {
  public double calculate(double position, double velocity);

  public double calculate(double position, double velocity, double nextVelocity, double dt);

  public double calculate(double position, double velocity, double acceleration);

  public void setGains(double ks, double kg, double kv, double ka);
}
