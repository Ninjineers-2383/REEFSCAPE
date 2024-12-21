package frc.robot.subsystems.position_joint;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.position_joint.PositionJointConstants.GravityType;
import frc.robot.subsystems.position_joint.PositionJointConstants.PositionJointGains;
import frc.robot.subsystems.position_joint.PositionJointConstants.PositionJointHardwareConfig;
import java.util.ArrayList;

public class PositionJointIOTalonFX implements PositionJointIO {
  private final String name;

  private final PositionJointHardwareConfig hardwareConfig;

  private final TalonFX[] motors;
  private final TalonFXConfiguration leaderConfig;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0);

  private final StatusSignal<AngularVelocity> velocity;

  private final ArrayList<StatusSignal<Angle>> positions = new ArrayList<>();
  private final ArrayList<StatusSignal<AngularVelocity>> velocities = new ArrayList<>();

  private final ArrayList<StatusSignal<Voltage>> voltages = new ArrayList<>();
  private final ArrayList<StatusSignal<Current>> currents = new ArrayList<>();

  private final boolean[] motorsConnected;

  private final double[] motorPositions;
  private final double[] motorVelocities;

  private final double[] motorVoltages;
  private final double[] motorCurrents;

  private final Alert[] motorAlerts;

  private double positionSetpoint = 0.0;

  public PositionJointIOTalonFX(String name, PositionJointHardwareConfig config) {
    this.name = name;
    hardwareConfig = config;

    assert config.canIds().length > 0 && (config.canIds().length == config.reversed().length);

    motors = new TalonFX[config.canIds().length];
    motorsConnected = new boolean[config.canIds().length];
    motorPositions = new double[config.canIds().length];
    motorVelocities = new double[config.canIds().length];
    motorVoltages = new double[config.canIds().length];
    motorCurrents = new double[config.canIds().length];
    motorAlerts = new Alert[config.canIds().length];

    motors[0] = new TalonFX(config.canIds()[0], config.canBus());
    leaderConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(
                        config.reversed()[0]
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(config.gearRatio())
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
            .withSlot0(null);

    tryUntilOk(5, () -> motors[0].getConfigurator().apply(leaderConfig));

    velocity = motors[0].getVelocity();
    positions.add(motors[0].getPosition());
    velocities.add(motors[0].getVelocity());

    voltages.add(motors[0].getSupplyVoltage());
    currents.add(motors[0].getStatorCurrent());

    motorAlerts[0] =
        new Alert(
            name + " Leader Motor Disconnected! CAN ID: " + config.canIds()[0], AlertType.kError);

    for (int i = 1; i < config.canIds().length; i++) {
      motors[i] = new TalonFX(config.canIds()[i], config.canBus());
      motors[i].setControl(new Follower(i, config.reversed()[i]));

      motorAlerts[i] =
          new Alert(
              name + " Follower Motor " + i + " Disconnected! CAN ID: " + config.canIds()[i],
              AlertType.kError);

      positions.add(motors[i].getPosition());
      velocities.add(motors[i].getVelocity());

      voltages.add(motors[i].getSupplyVoltage());
      currents.add(motors[i].getStatorCurrent());
    }
  }

  @Override
  public void updateInputs(PositionJointIOInputs inputs) {
    inputs.velocity = motors[0].getVelocity().getValueAsDouble();

    inputs.desiredVelocity = positionSetpoint;

    for (int i = 0; i < motors.length; i++) {
      motorsConnected[i] =
          BaseStatusSignal.refreshAll(
                  velocity, positions.get(i), velocities.get(i), voltages.get(i), currents.get(i))
              .isOK();

      motorPositions[i] = positions.get(i).getValueAsDouble();
      motorVelocities[i] = velocities.get(i).getValueAsDouble();

      motorVoltages[i] = voltages.get(i).getValueAsDouble();
      motorCurrents[i] = motors[i].getStatorCurrent().getValueAsDouble();

      motorAlerts[i].set(motorsConnected[i]);
    }

    inputs.motorsConnected = motorsConnected;

    inputs.motorPositions = motorPositions;
    inputs.motorVelocities = motorVelocities;

    inputs.motorVoltages = motorVoltages;
    inputs.motorCurrents = motorCurrents;
  }

  @Override
  public void setPosition(double position, double velocity) {
    positionSetpoint = position;

    motors[0].setControl(positionRequest.withPosition(position).withVelocity(velocity));
  }

  @Override
  public void setVoltage(double voltage) {
    motors[0].setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void setGains(PositionJointGains gains) {
    GravityTypeValue gravity;
    if (hardwareConfig.gravity() == GravityType.CONSTANT) {
      gravity = GravityTypeValue.Elevator_Static;
    } else if (hardwareConfig.gravity() == GravityType.COSINE) {
      gravity = GravityTypeValue.Arm_Cosine;
    } else {
      throw new IllegalArgumentException("SIN gravity is not supported for TalonFX");
    }

    motors[0]
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withKP(gains.kP())
                .withKI(gains.kI())
                .withKD(gains.kD())
                .withKV(gains.kV())
                .withKA(gains.kA())
                .withKS(gains.kS())
                .withKG(gains.kG())
                .withGravityType(gravity));

    System.out.println(name + " gains set to " + gains);
  }

  @Override
  public String getName() {
    return name;
  }
}
