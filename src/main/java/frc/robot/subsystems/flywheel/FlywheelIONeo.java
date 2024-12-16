package frc.robot.subsystems.flywheel;

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
import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelGains;
import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelHardwareConfig;

public class FlywheelIONeo implements FlywheelIO {
  private final String name;

  private final SparkMax[] motors = new SparkMax[] {};

  private SparkBaseConfig leaderConfig;
  private SparkBaseConfig followerConfig;

  private double velocitySetpoint = 0.0;

  private boolean[] motorsConnected;

  private double[] motorPositions;
  private double[] motorVelocities;

  private double[] motorVoltages;
  private double[] motorCurrents;

  private Alert[] motorAlerts;

  public FlywheelIONeo(String name, FlywheelHardwareConfig config) {
    this.name = name;

    motors[0] = new SparkMax(config.canIds()[0], MotorType.kBrushless);
    leaderConfig = new SparkMaxConfig();
    leaderConfig
        .encoder
        .positionConversionFactor(config.gearRatio())
        .velocityConversionFactor(config.gearRatio());

    motors[0].configure(
        leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    motors[0].setInverted(config.reversed()[0]);

    motorAlerts[0] =
        new Alert(
            name + " Leader Motor Disconnected! CAN ID: " + config.canIds()[0], AlertType.kError);

    followerConfig = new SparkMaxConfig().follow(motors[0]);

    for (int i = 1; i < config.canIds().length; i++) {
      motors[i] = new SparkMax(config.canIds()[i], MotorType.kBrushless);
      motors[i].configure(
          followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      motors[i].setInverted(config.reversed()[i]);

      motorAlerts[i] =
          new Alert(
              name + " Follower Motor " + i + " Disconnected! CAN ID: " + config.canIds()[i],
              AlertType.kError);
    }
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocity = motors[0].getEncoder().getVelocity();

    inputs.velocitySetpoint = velocitySetpoint;

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
  public void setVelocity(double velocity) {
    velocitySetpoint = velocity;

    motors[0].getClosedLoopController().setReference(velocitySetpoint, ControlType.kVelocity, 0);
  }

  @Override
  public void setVoltage(double voltage) {
    motors[0].setVoltage(voltage);
  }

  @Override
  public void setGains(FlywheelGains gains) {
    motors[0].configure(
        leaderConfig.apply(
            new ClosedLoopConfig().pidf(gains.kP(), gains.kI(), gains.kD(), gains.kV())),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  @Override
  public String getName() {
    return name;
  }
}
