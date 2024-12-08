package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelGains;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class FlywheelSubsystem extends SubsystemBase {
  private final FlywheelIO m_flywheel;
  private final FlywheelIOInputsAutoLogged m_inputs = new FlywheelIOInputsAutoLogged();

  private final FlywheelGains m_gains;

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kA;

  public FlywheelSubsystem(FlywheelIO io, FlywheelGains gains) {
    m_flywheel = io;

    m_gains = gains;

    kP = new LoggedTunableNumber(m_gains.name() + "/Gains/kP", gains.kP());
    kI = new LoggedTunableNumber(m_gains.name() + "/Gains/kI", gains.kI());
    kD = new LoggedTunableNumber(m_gains.name() + "/Gains/kD", gains.kD());
    kS = new LoggedTunableNumber(m_gains.name() + "/Gains/kS", gains.kS());
    kV = new LoggedTunableNumber(m_gains.name() + "/Gains/kV", gains.kV());
    kA = new LoggedTunableNumber(m_gains.name() + "/Gains/kA", gains.kA());

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          io.setGains(
              new FlywheelGains(
                  m_gains.name(),
                  values[0],
                  values[1],
                  values[2],
                  values[3],
                  values[4],
                  values[5]));
        },
        kP,
        kI,
        kD,
        kS,
        kV,
        kA);
  }

  @Override
  public void periodic() {
    m_flywheel.updateInputs(m_inputs);
    Logger.processInputs(m_gains.name(), m_inputs);
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
    return m_inputs.velocitySetpoint;
  }
}
