package frc.robot.subsystems.drive.spark;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.drive.DriveConstants;

public class SparkMaxModuleConstants {
  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.1;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn PID configuration
  public static final double turnKp = 2.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  public static final SparkBaseConfig driveMotorConfig =
      new SparkMaxConfig()
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(DriveConstants.driveMotorCurrentLimit)
          .voltageCompensation(12.0)
          .apply(
              new EncoderConfig()
                  .positionConversionFactor(DriveConstants.driveEncoderPositionFactor)
                  .velocityConversionFactor(DriveConstants.driveEncoderVelocityFactor)
                  .uvwMeasurementPeriod(10)
                  .uvwAverageDepth(2))
          .apply(
              new ClosedLoopConfig()
                  .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                  .pidf(
                      driveKp, 0.0,
                      driveKd, 0.0))
          .apply(
              new SignalsConfig()
                  .primaryEncoderPositionAlwaysOn(true)
                  .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
                  .primaryEncoderVelocityAlwaysOn(true)
                  .primaryEncoderVelocityPeriodMs(20)
                  .appliedOutputPeriodMs(20)
                  .busVoltagePeriodMs(20)
                  .outputCurrentPeriodMs(20));

  public static final SparkBaseConfig turnMotorConfig =
      new SparkMaxConfig()
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(DriveConstants.turnMotorCurrentLimit)
          .voltageCompensation(12.0)
          .apply(
              new EncoderConfig()
                  .positionConversionFactor(DriveConstants.turnEncoderPositionFactor)
                  .velocityConversionFactor(DriveConstants.turnEncoderVelocityFactor)
                  .uvwMeasurementPeriod(10)
                  .uvwAverageDepth(2))
          .apply(
              new ClosedLoopConfig()
                  .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                  .positionWrappingEnabled(true)
                  .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
                  .pidf(turnKp, 0.0, turnKd, 0.0))
          .apply(
              new SignalsConfig()
                  .primaryEncoderPositionAlwaysOn(true)
                  .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
                  .primaryEncoderVelocityAlwaysOn(true)
                  .primaryEncoderVelocityPeriodMs(20)
                  .appliedOutputPeriodMs(20)
                  .busVoltagePeriodMs(20)
                  .outputCurrentPeriodMs(20));

  public static final CANcoderConfiguration cancoderConfig =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

  public record ModuleSpecificConfiguration(
      int driveCanID,
      int steerCanID,
      int CANcoderID,
      Angle CANCoderOffset,
      boolean invertDrive,
      boolean invertSteer) {}

  public static final ModuleSpecificConfiguration frontLeft =
      new ModuleSpecificConfiguration(31, 41, 21, Rotations.of(-0.181640625 + 0.5), false, false);
  public static final ModuleSpecificConfiguration frontRight =
      new ModuleSpecificConfiguration(32, 42, 22, Rotations.of(0.169678 - 0.5), false, false);
  public static final ModuleSpecificConfiguration rearLeft =
      new ModuleSpecificConfiguration(33, 43, 23, Rotations.of(-0.328857 + 0.5), false, true);
  public static final ModuleSpecificConfiguration rearRight =
      new ModuleSpecificConfiguration(34, 44, 24, Rotations.of(0.055908 - 0.5), true, false);
}
