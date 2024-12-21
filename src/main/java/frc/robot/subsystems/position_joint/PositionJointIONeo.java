package frc.robot.subsystems.position_joint;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.position_joint.PositionJointConstants.GravityType;
import frc.robot.subsystems.position_joint.PositionJointConstants.PositionJointGains;
import frc.robot.subsystems.position_joint.PositionJointConstants.PositionJointHardwareConfig;
import frc.robot.util.PositionJointFeedforward;
import frc.robot.util.TunableArmFeedforward;
import frc.robot.util.TunableElevatorFeedforward;

public class PositionJointIONeo implements PositionJointIO {
  private final String name;

  private final SparkMax[] motors;
  private final SparkBaseConfig leaderConfig;

  private final boolean[] motorsConnected;

  private final double[] motorPositions;
  private final double[] motorVelocities;

  private final double[] motorVoltages;
  private final double[] motorCurrents;

  private final Alert[] motorAlerts;

  private final PositionJointFeedforward feedforward;
  private final double feedforward_position_addition;

  private double currentPosition = 0.0;
  private double positionSetpoint = 0.0;
  private double velocitySetpoint = 0.0;

  public PositionJointIONeo(String name, PositionJointHardwareConfig config) {
    this.name = name;

    assert config.canIds().length > 0 && (config.canIds().length == config.reversed().length);

    motors = new SparkMax[config.canIds().length];
    motorsConnected = new boolean[config.canIds().length];
    motorPositions = new double[config.canIds().length];
    motorVelocities = new double[config.canIds().length];
    motorVoltages = new double[config.canIds().length];
    motorCurrents = new double[config.canIds().length];
    motorAlerts = new Alert[config.canIds().length];

    motors[0] = new SparkMax(config.canIds()[0], MotorType.kBrushless);
    leaderConfig =
        new SparkMaxConfig()
            .inverted(config.reversed()[0])
            .idleMode(IdleMode.kBrake)
            .apply(
                new EncoderConfig()
                    .positionConversionFactor(config.gearRatio())
                    .velocityConversionFactor(config.gearRatio()));

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

    if (config.gravity() == GravityType.CONSTANT) {
      feedforward = new TunableElevatorFeedforward();
      feedforward_position_addition = 0.0;
    } else {
      feedforward = new TunableArmFeedforward();
      if (config.gravity() == GravityType.SIN) {
        feedforward_position_addition = -Math.PI / 2;
      } else {
        feedforward_position_addition = 0.0;
      }
    }
  }

  @Override
  public void updateInputs(PositionJointIOInputs inputs) {
    currentPosition = motors[0].getEncoder().getPosition();
    inputs.position = currentPosition;

    inputs.desiredPosition = positionSetpoint;
    inputs.desiredVelocity = velocitySetpoint;

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
  public void setPosition(double desiredPosition, double desiredVelocity) {
    positionSetpoint = desiredPosition;

    double ffposition = currentPosition + feedforward_position_addition;

    motors[0]
        .getClosedLoopController()
        .setReference(
            positionSetpoint,
            ControlType.kPosition,
            0,
            feedforward.calculate(ffposition, velocitySetpoint, desiredPosition, 0.02));

    velocitySetpoint = desiredVelocity;
  }

  @Override
  public void setVoltage(double voltage) {
    motors[0].setVoltage(voltage);
  }

  @Override
  public void setGains(PositionJointGains gains) {
    feedforward.setGains(gains.kS(), gains.kG(), gains.kV(), gains.kA());

    motors[0].configure(
        leaderConfig.apply(new ClosedLoopConfig().pidf(gains.kP(), gains.kI(), gains.kD(), 0)),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

    System.out.println(name + " gains set to " + gains);
  }

  @Override
  public String getName() {
    return name;
  }
}
