package frc.robot.subsystems.drive.talon;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.drive.DriveConstants;

public class TalonFXModuleConstants {
  public static String CANBusName = "CANBus";

  public static final Slot0Configs drivePIDConfig = new Slot0Configs();

  public static final Current slipCurrent = Amps.of(60.0);

  public static final ClosedLoopOutputType driveMotorClosedLoopOutput =
      ClosedLoopOutputType.TorqueCurrentFOC;

  public static final TalonFXConfiguration driveMotorConfig =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
          .withSlot0(drivePIDConfig)
          .withFeedback(
              new FeedbackConfigs().withSensorToMechanismRatio(DriveConstants.driveMotorGearRatio))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(slipCurrent.magnitude())
                  .withPeakReverseTorqueCurrent(-slipCurrent.magnitude()));

  private static final Slot0Configs turnPIDConfig = new Slot0Configs();
  private static final ClosedLoopGeneralConfigs ContinuousWrap = new ClosedLoopGeneralConfigs();

  {
    ContinuousWrap.ContinuousWrap = true;
  }

  public static final ClosedLoopOutputType steerMotorClosedLoopOutput =
      ClosedLoopOutputType.Voltage;

  public static final TalonFXConfiguration turnMotorConfig =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
          .withSlot0(turnPIDConfig)
          .withFeedback(
              new FeedbackConfigs()
                  .withRotorToSensorRatio(DriveConstants.steerMotorGearRatio)
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(100.0 / DriveConstants.steerMotorGearRatio)
                  .withMotionMagicAcceleration(1000.0 / DriveConstants.steerMotorGearRatio))
          .withClosedLoopGeneral(ContinuousWrap)
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(slipCurrent.magnitude())
                  .withPeakReverseTorqueCurrent(-slipCurrent.magnitude()));

  public static final CANcoderConfiguration cancoderConfig =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

  public record ModuleSpecificConfiguration(
      int driveCanID, int steerCanID, int CANcoderID, Angle CANCoderOffset) {}

  public static final ModuleSpecificConfiguration frontLeft =
      new ModuleSpecificConfiguration(0, 1, 2, Rotations.of(0.0));
  public static final ModuleSpecificConfiguration frontRight =
      new ModuleSpecificConfiguration(0, 1, 2, Rotations.of(0.0));
  public static final ModuleSpecificConfiguration rearLeft =
      new ModuleSpecificConfiguration(0, 1, 2, Rotations.of(0.0));
  public static final ModuleSpecificConfiguration rearRight =
      new ModuleSpecificConfiguration(0, 1, 2, Rotations.of(0.0));
}
