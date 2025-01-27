package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelGains;
import frc.robot.util.mechanical_advantage.LinearProfile;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO flywheel;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private final String name;

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kA;

  private final LoggedTunableNumber kMaxAccel;

  private final LoggedTunableNumber kTolerance;

  private final LoggedTunableNumber kSetpoint;

  private final LinearProfile profile;
  private double velocitySetpoint;

  public Flywheel(FlywheelIO io, FlywheelGains gains) {
    flywheel = io;

    name = io.getName();

    kP = new LoggedTunableNumber(name + "/Gains/kP", gains.kP());
    kI = new LoggedTunableNumber(name + "/Gains/kI", gains.kI());
    kD = new LoggedTunableNumber(name + "/Gains/kD", gains.kD());
    kS = new LoggedTunableNumber(name + "/Gains/kS", gains.kS());
    kV = new LoggedTunableNumber(name + "/Gains/kV", gains.kV());
    kA = new LoggedTunableNumber(name + "/Gains/kA", gains.kA());

    kMaxAccel = new LoggedTunableNumber(name + "/Gains/kMaxAccel", gains.kMaxAccel());

    kTolerance = new LoggedTunableNumber(name + "/Gains/kTolerance", gains.kTolerance());

    kSetpoint = new LoggedTunableNumber(name + "/Gains/kSetpoint", 0.0);

    profile = new LinearProfile(gains.kMaxAccel(), 0.02);
  }

  @Override
  public void periodic() {
    flywheel.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    velocitySetpoint = profile.calculateSetpoint();
    flywheel.setVelocity(velocitySetpoint);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          flywheel.setGains(
              new FlywheelGains(
                  values[0], values[1], values[2], values[3], values[4], values[5], values[6],
                  values[7]));

          profile.setGoal(values[8], velocitySetpoint);

          profile.setMaxAcceleration(values[6]);
        },
        kP,
        kI,
        kD,
        kS,
        kV,
        kA,
        kMaxAccel,
        kTolerance,
        kSetpoint);
  }

  public void setVelocity(double velocity) {
    profile.setGoal(velocity, velocitySetpoint);
  }

  public void setVoltage(double voltage) {
    flywheel.setVoltage(voltage);
  }

  public double getVelocity() {
    return inputs.velocity;
  }

  public double getVelocitySetpoint() {
    return inputs.desiredVelocity;
  }

  public boolean isFinished() {
    return Math.abs(inputs.velocity - inputs.desiredVelocity) < kTolerance.get();
  }
}
