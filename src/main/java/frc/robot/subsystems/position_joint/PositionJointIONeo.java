package frc.robot.subsystems.position_joint;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.position_joint.PositionJointConstants.GravityType;
import frc.robot.subsystems.position_joint.PositionJointConstants.PositionJointGains;
import frc.robot.subsystems.position_joint.PositionJointConstants.PositionJointHardwareConfig;
import frc.robot.util.TunableArmFeedforward;

public class PositionJointIONeo implements PositionJointIO {
  private final String name;
  private final PositionJointHardwareConfig config;

  private final SparkMax[] motors = new SparkMax[] {};

  private TunableArmFeedforward feedforward = new TunableArmFeedforward(0, 0, 0);

  private SparkBaseConfig leaderConfig;

  private double position = 0.0;
  private double positionSetpoint = 0.0;

  private boolean[] motorsConnected;

  private double[] motorPositions;
  private double[] motorVelocities;

  private double[] motorVoltages;
  private double[] motorCurrents;

  private Alert[] motorAlerts;

  public PositionJointIONeo(String name, PositionJointHardwareConfig config) {
    this.name = name;
    this.config = config;

    motors[0] = new SparkMax(config.canIds()[0], MotorType.kBrushless);
    leaderConfig = new SparkMaxConfig().inverted(config.reversed()[0]);
    leaderConfig
        .encoder
        .positionConversionFactor(config.gearRatio())
        .velocityConversionFactor(config.gearRatio());

    motors[0].configure(
        leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    motorAlerts[0] =
        new Alert(
            name + " Leader Motor Disconnected! CAN ID: " + config.canIds()[0], AlertType.kError);

    for (int i = 1; i < config.canIds().length; i++) {
      motors[i] = new SparkMax(config.canIds()[i], MotorType.kBrushless);
      motors[i].configure(
          new SparkMaxConfig().follow(motors[0]).inverted(config.reversed()[i]),
          ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);

      motorAlerts[i] =
          new Alert(
              name + " Follower Motor " + i + " Disconnected! CAN ID: " + config.canIds()[i],
              AlertType.kError);
    }
  }

  @Override
  public void updateInputs(PositionJointIOInputs inputs) {
    position = motors[0].getEncoder().getPosition();
    inputs.position = position;

    inputs.desiredPosition = positionSetpoint;

    for (int i = 0; i < motors.length; i++) {
      motorsConnected[i] = motors[i].getLastError() == REVLibError.kOk;

      motorPositions[i] = motors[i].getEncoder().getPosition();
      motorVelocities[i] = motors[i].getEncoder().getVelocity();

      motorVoltages[i] = motors[i].getAppliedOutput() * 12;
      motorCurrents[i] = motors[i].getOutputCurrent();

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

    motors[0]
        .getClosedLoopController()
        .setReference(
            position, ControlType.kPosition, 0, feedforward.calculate(getFFPosition(), velocity));
  }

  @Override
  public void setVoltage(double voltage) {
    motors[0].setVoltage(voltage);
  }

  @Override
  public void setGains(PositionJointGains gains) {
    feedforward.setGains(gains.kS(), gains.kG(), gains.kV(), gains.kA());

    motors[0].configure(
        leaderConfig.apply(
            new ClosedLoopConfig().pidf(gains.kP(), gains.kI(), gains.kD(), gains.kV())),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  private double getFFPosition() {
    if (config.gravity() == GravityType.CONSTANT) {
      return 0;
    } else {
      return position;
    }
  }

  @Override
  public String getName() {
    return name;
  }
}
