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
import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelGains;
import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelHardwareConfig;

public class FlywheelIONeo implements FlywheelIO {
  private final SparkMax[] motors = new SparkMax[] {};

  private double velocitySetpoint = 0.0;

  private boolean[] motorsConnected = {true};

  private double[] motorPositions = {0.0};
  private double[] motorVelocities = {0.0};

  private double[] motorVoltages = {0.0};
  private double[] motorCurrents = {0.0};

  public FlywheelIONeo(FlywheelHardwareConfig config) {
    motors[0] = new SparkMax(config.canIds()[0], MotorType.kBrushless);
    motors[0].setInverted(config.reversed()[0]);

    SparkBaseConfig followerConfig = new SparkMaxConfig().follow(motors[0]);

    for (int i = 1; i < config.canIds().length; i++) {
      motors[i] = new SparkMax(config.canIds()[i], MotorType.kBrushless);
      motors[i].setInverted(config.reversed()[i]);
      motors[i].configure(
          followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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
        new SparkMaxConfig().apply(new ClosedLoopConfig().pid(gains.kP(), gains.kI(), gains.kD())),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }
}
