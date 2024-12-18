package frc.robot.subsystems.position_joint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.position_joint.PositionJointConstants.PositionJointGains;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class PositionJoint extends SubsystemBase {
  private final PositionJointIO m_positionJoint;
  private final PositionJointIOInputsAutoLogged m_inputs = new PositionJointIOInputsAutoLogged();

  private final String m_name;

  private final PositionJointGains m_gains;

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

  private TrapezoidProfile.Constraints constraints;

  private TrapezoidProfile profile;

  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  public PositionJoint(PositionJointIO io, PositionJointGains gains) {
    m_positionJoint = io;
    m_name = io.getName();

    m_gains = gains;

    kP = new LoggedTunableNumber(m_name + "/Gains/kP", gains.kP());
    kI = new LoggedTunableNumber(m_name + "/Gains/kI", gains.kI());
    kD = new LoggedTunableNumber(m_name + "/Gains/kD", gains.kD());
    kS = new LoggedTunableNumber(m_name + "/Gains/kS", gains.kS());
    kG = new LoggedTunableNumber(m_name + "/Gains/kG", gains.kG());
    kV = new LoggedTunableNumber(m_name + "/Gains/kV", gains.kV());
    kA = new LoggedTunableNumber(m_name + "/Gains/kA", gains.kA());

    kMaxVelo = new LoggedTunableNumber(m_name + "/Gains/kMaxVelo", gains.kMaxVelo());
    kMaxAccel = new LoggedTunableNumber(m_name + "/Gains/kMaxAccel", gains.kMaxAccel());

    kMinPosition = new LoggedTunableNumber(m_name + "/Gains/kMinPosition", m_gains.kMinPosition());
    kMaxPosition = new LoggedTunableNumber(m_name + "/Gains/kMaxPosition", m_gains.kMaxPosition());

    kTolerance = new LoggedTunableNumber(m_name + "/Gains/kTolerance", m_gains.kTolerance());

    constraints = new TrapezoidProfile.Constraints(gains.kMaxVelo(), gains.kMaxAccel());
    profile = new TrapezoidProfile(constraints);

    goal = new TrapezoidProfile.State(getPosition(), 0);
    setpoint = goal;
  }

  @Override
  public void periodic() {
    m_positionJoint.updateInputs(m_inputs);
    Logger.processInputs(m_name, m_inputs);

    setpoint = profile.calculate(0.02, setpoint, goal);

    m_positionJoint.setPosition(setpoint.position, setpoint.velocity);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          m_positionJoint.setGains(
              new PositionJointGains(
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

          constraints = new TrapezoidProfile.Constraints(values[7], values[8]);
          profile = new TrapezoidProfile(constraints);
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
        kMaxPosition,
        kTolerance);

    Logger.recordOutput(m_name + "/isFinished", isFinished());
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
    m_positionJoint.setVoltage(voltage);
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
