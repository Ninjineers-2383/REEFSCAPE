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
  public static String CANBusName = "Drive";

  public static final Slot0Configs drivePIDConfig =
      new Slot0Configs().withKP(2).withKI(0).withKD(0).withKS(0.23).withKV(0.7).withKA(0.007);

  public static final Current slipCurrent = Amps.of(60.0);

  public static final ClosedLoopOutputType driveMotorClosedLoopOutput =
      ClosedLoopOutputType.Voltage;

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

  private static final Slot0Configs turnPIDConfig =
      new Slot0Configs().withKP(30).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
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
