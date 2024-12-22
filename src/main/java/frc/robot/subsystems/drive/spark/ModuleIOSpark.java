// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive.spark;

import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.spark.SparkMaxModuleConstants.ModuleSpecificConfiguration;
import frc.robot.subsystems.drive.talon.TalonFXModuleConstants;
import frc.robot.util.PhoenixUtil;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Max drive motor controller, Spark Max turn motor controller,
 * and CANCoder.
 */
public class ModuleIOSpark implements ModuleIO {
  // Hardware objects
  private final SparkBase driveSpark;
  private final SparkBase turnSpark;
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;
  private final CANcoder cancoder;

  // Hardware config
  private final SparkBaseConfig driveConfig;
  private final SparkBaseConfig turnConfig;

  // Closed loop controllers
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  private ModuleGains gains;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Input from CANCoder
  private final StatusSignal<Angle> turnAbsolutePosition;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  public ModuleIOSpark(ModuleSpecificConfiguration constants) {
    driveSpark = new SparkMax(constants.driveCanID(), MotorType.kBrushless);
    turnSpark = new SparkMax(constants.steerCanID(), MotorType.kBrushless);

    driveEncoder = driveSpark.getEncoder();
    turnEncoder = turnSpark.getEncoder();
    cancoder = new CANcoder(constants.CANcoderID());

    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    // Configure drive motor
    driveConfig = SparkMaxModuleConstants.driveMotorConfig;
    driveConfig.inverted(constants.invertDrive());
    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

    // Configure CANCoder
    CANcoderConfiguration cancoderConfig = TalonFXModuleConstants.cancoderConfig;
    cancoderConfig.MagnetSensor.withMagnetOffset(constants.CANCoderOffset());
    PhoenixUtil.tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig, 0.25));

    // Configure turn motor
    turnConfig = SparkMaxModuleConstants.turnMotorConfig;
    turnConfig.inverted(constants.invertSteer());
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        turnSpark,
        5,
        () -> turnEncoder.setPosition(cancoder.getAbsolutePosition().getValueAsDouble()));

    // Create odometry queues
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
    turnPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);

    // Create CANCoder status signal
    turnAbsolutePosition = cancoder.getAbsolutePosition();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, turnAbsolutePosition);

    gains =
        new ModuleGains(
            SparkMaxModuleConstants.driveKp,
            0.0,
            SparkMaxModuleConstants.driveKd,
            SparkMaxModuleConstants.driveKs,
            SparkMaxModuleConstants.driveKv,
            0.0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    sparkStickyFault = false;
    ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
    ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
    ifOk(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update turn inputs
    sparkStickyFault = false;
    ifOk(
        turnSpark,
        turnEncoder::getPosition,
        (value) -> inputs.turnPosition = new Rotation2d(value));
    ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts =
        SparkMaxModuleConstants.driveKs * Math.signum(velocityRadPerSec)
            + SparkMaxModuleConstants.driveKv * velocityRadPerSec;
    driveController.setReference(
        velocityRadPerSec, ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.getRadians(),
            SparkMaxModuleConstants.turnPIDMinInput,
            SparkMaxModuleConstants.turnPIDMaxInput);
    turnController.setReference(setpoint, ControlType.kPosition);
  }

  @Override
  public ModuleGains getGains() {
    return gains;
  }

  @Override
  public void setGains(ModuleGains gains) {
    this.gains = gains;

    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig.apply(
                    new ClosedLoopConfig().pidf(gains.kP(), gains.kI(), gains.kD(), gains.kV())),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters));
  }
}
