package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorGains;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO m_elevator;
  private final ElevatorIOInputsAutoLogged m_inputs = new ElevatorIOInputsAutoLogged();

  private final ElevatorGains m_gains;

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kG;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kA;

  private final LoggedTunableNumber kMaxVelo;
  private final LoggedTunableNumber kMaxAccel;

  private final LoggedTunableNumber kMinPosition;
  private final LoggedTunableNumber kMaxPosition;

  private final LoggedTunableNumber kTolerance;

  private final TrapezoidProfile.Constraints constraints;

  private TrapezoidProfile profile;

  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  public Elevator(ElevatorIO io, ElevatorGains gains) {
    m_elevator = io;
    m_gains = gains;

    kP = new LoggedTunableNumber(m_gains.name() + "/Gains/kP", gains.kP());
    kI = new LoggedTunableNumber(m_gains.name() + "/Gains/kI", gains.kI());
    kD = new LoggedTunableNumber(m_gains.name() + "/Gains/kD", gains.kD());
    kS = new LoggedTunableNumber(m_gains.name() + "/Gains/kS", gains.kS());
    kG = new LoggedTunableNumber(m_gains.name() + "/Gains/kG", gains.kG());
    kV = new LoggedTunableNumber(m_gains.name() + "/Gains/kV", gains.kV());
    kA = new LoggedTunableNumber(m_gains.name() + "/Gains/kA", gains.kA());

    kMaxVelo = new LoggedTunableNumber(m_gains.name() + "/Gains/kMaxVelo", gains.kMaxVelo());
    kMaxAccel = new LoggedTunableNumber(m_gains.name() + "/Gains/kMaxAccel", gains.kMaxAccel());

    kMinPosition =
        new LoggedTunableNumber(m_gains.name() + "/Gains/kMinPosition", m_gains.kMinPosition());
    kMaxPosition =
        new LoggedTunableNumber(m_gains.name() + "/Gains/kMaxPosition", m_gains.kMaxPosition());

    kTolerance =
        new LoggedTunableNumber(m_gains.name() + "/Gains/kTolerance", m_gains.kTolerance());

    constraints = new TrapezoidProfile.Constraints(gains.kMaxVelo(), gains.kMaxAccel());
    profile = new TrapezoidProfile(constraints);

    goal = new TrapezoidProfile.State(getPosition(), 0);
    setpoint = goal;

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          io.setGains(
              new ElevatorGains(
                  m_gains.name(),
                  values[0],
                  values[1],
                  values[2],
                  values[3],
                  values[4],
                  values[5],
                  values[6],
                  values[7],
                  values[8],
                  values[9],
                  values[10],
                  values[11]));
        },
        kP,
        kI,
        kD,
        kS,
        kG,
        kV,
        kA,
        kMaxVelo,
        kMaxAccel,
        kMinPosition,
        kMaxPosition);
  }

  @Override
  public void periodic() {
    m_elevator.updateInputs(m_inputs);
    Logger.processInputs(m_gains.name(), m_inputs);

    setpoint = profile.calculate(0.02, setpoint, goal);

    m_elevator.setPosition(setpoint.position, setpoint.velocity);
  }

  public void setPosition(double position) {
    goal =
        new TrapezoidProfile.State(
            MathUtil.clamp(position, kMinPosition.get(), kMaxPosition.get()), 0);
  }

  public void incrementPosition(double deltaPosition) {
    goal.position += deltaPosition;
  }

  public void setVoltage(double voltage) {
    m_elevator.setVoltage(voltage);
  }

  public double getPosition() {
    return m_inputs.position;
  }

  public double getDesiredPosition() {
    return m_inputs.desiredPosition;
  }

  public boolean isFinished() {
    return Math.abs(m_inputs.position - goal.position) < kTolerance.get();
  }
}
