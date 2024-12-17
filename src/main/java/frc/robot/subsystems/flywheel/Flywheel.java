package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelGains;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO m_flywheel;
  private final FlywheelIOInputsAutoLogged m_inputs = new FlywheelIOInputsAutoLogged();

  private final String m_name;

  private final FlywheelGains m_gains;

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kA;

  public Flywheel(FlywheelIO io, FlywheelGains gains) {
    m_flywheel = io;

    m_name = io.getName();

    m_gains = gains;

    kP = new LoggedTunableNumber(m_name + "/Gains/kP", m_gains.kP());
    kI = new LoggedTunableNumber(m_name + "/Gains/kI", m_gains.kI());
    kD = new LoggedTunableNumber(m_name + "/Gains/kD", m_gains.kD());
    kS = new LoggedTunableNumber(m_name + "/Gains/kS", m_gains.kS());
    kV = new LoggedTunableNumber(m_name + "/Gains/kV", m_gains.kV());
    kA = new LoggedTunableNumber(m_name + "/Gains/kA", m_gains.kA());
  }

  @Override
  public void periodic() {
    m_flywheel.updateInputs(m_inputs);
    Logger.processInputs(m_name, m_inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          m_flywheel.setGains(
              new FlywheelGains(values[0], values[1], values[2], values[3], values[4], values[5]));
        },
        kP,
        kI,
        kD,
        kS,
        kV,
        kA);
  }

  public void setVelocity(double velocity) {
    m_flywheel.setVelocity(velocity);
  }

  public void setVoltage(double voltage) {
    m_flywheel.setVoltage(voltage);
  }

  public double getVelocity() {
    return m_inputs.velocity;
  }

  public double getVelocitySetpoint() {
    return m_inputs.desiredVelocity;
  }
}
