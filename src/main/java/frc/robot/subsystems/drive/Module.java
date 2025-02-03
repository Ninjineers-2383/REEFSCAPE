package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorConstants.AzimuthMotorGains;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIO;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIOInputsAutoLogged;
import frc.robot.subsystems.drive.drive_motor.DriveMotorConstants.DriveMotorGains;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIO;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIOInputsAutoLogged;
import frc.robot.util.OnboardModuleState;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final DriveMotorIO driveMotor;
  private final DriveMotorIOInputsAutoLogged driveInputs = new DriveMotorIOInputsAutoLogged();

  private final AzimuthMotorIO azimuthMotor;
  private final AzimuthMotorIOInputsAutoLogged azimuthInputs = new AzimuthMotorIOInputsAutoLogged();

  private final String driveName;
  private final String azimuthName;

  private final LoggedTunableNumber drivekP;
  private final LoggedTunableNumber drivekI;
  private final LoggedTunableNumber drivekD;
  private final LoggedTunableNumber drivekS;
  private final LoggedTunableNumber drivekV;
  private final LoggedTunableNumber drivekA;

  private final LoggedTunableNumber drivekMaxAccel;

  private final LoggedTunableNumber azimuthkP;
  private final LoggedTunableNumber azimuthkI;
  private final LoggedTunableNumber azimuthkD;
  private final LoggedTunableNumber azimuthkS;
  private final LoggedTunableNumber azimuthkV;
  private final LoggedTunableNumber azimuthkA;

  private final LoggedTunableNumber azimuthkMaxVelo;
  private final LoggedTunableNumber azimuthkMaxAccel;

  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(
      DriveMotorIO driveMotorIO,
      DriveMotorGains driveGains,
      AzimuthMotorIO azimuthMotorIO,
      AzimuthMotorGains azimuthGains) {
    driveMotor = driveMotorIO;
    azimuthMotor = azimuthMotorIO;

    driveName = driveMotorIO.getName();
    azimuthName = azimuthMotorIO.getName();

    drivekP = new LoggedTunableNumber("Drive/" + driveName + "/Gains/kP", driveGains.kP());
    drivekI = new LoggedTunableNumber("Drive/" + driveName + "/Gains/kI", driveGains.kI());
    drivekD = new LoggedTunableNumber("Drive/" + driveName + "/Gains/kD", driveGains.kD());
    drivekS = new LoggedTunableNumber("Drive/" + driveName + "/Gains/kS", driveGains.kS());
    drivekV = new LoggedTunableNumber("Drive/" + driveName + "/Gains/kV", driveGains.kV());
    drivekA = new LoggedTunableNumber("Drive/" + driveName + "/Gains/kA", driveGains.kA());

    drivekMaxAccel =
        new LoggedTunableNumber("Drive/" + driveName + "/Gains/kMaxAccel", driveGains.kMaxAccel());

    azimuthkP = new LoggedTunableNumber("Drive/" + azimuthName + "/Gains/kP", azimuthGains.kP());
    azimuthkI = new LoggedTunableNumber("Drive/" + azimuthName + "/Gains/kI", azimuthGains.kI());
    azimuthkD = new LoggedTunableNumber("Drive/" + azimuthName + "/Gains/kD", azimuthGains.kD());
    azimuthkS = new LoggedTunableNumber("Drive/" + azimuthName + "/Gains/kS", azimuthGains.kS());
    azimuthkV = new LoggedTunableNumber("Drive/" + azimuthName + "/Gains/kV", azimuthGains.kV());
    azimuthkA = new LoggedTunableNumber("Drive/" + azimuthName + "/Gains/kA", azimuthGains.kA());

    azimuthkMaxVelo =
        new LoggedTunableNumber(
            "Drive/" + azimuthName + "/Gains/kMaxVelo", azimuthGains.kMaxVelo());
    azimuthkMaxAccel =
        new LoggedTunableNumber(
            "Drive/" + azimuthName + "/Gains/kMaxAccel", azimuthGains.kMaxAccel());
  }

  public void periodic() {
    driveMotor.updateInputs(driveInputs);
    Logger.processInputs(driveName, driveInputs);

    azimuthMotor.updateInputs(azimuthInputs);
    Logger.processInputs(azimuthName, azimuthInputs);

    // Calculate positions for odometry
    int sampleCount = driveInputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters =
          driveInputs.odometryDrivePositionsRad[i] * DriveConstants.driveWheelRadiusMeters;
      Rotation2d angle = azimuthInputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          driveMotor.setGains(
              new DriveMotorGains(
                  values[0], values[1], values[2], values[3], values[4], values[5], values[6]));
        },
        drivekP,
        drivekI,
        drivekD,
        drivekS,
        drivekV,
        drivekA,
        drivekMaxAccel);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          azimuthMotor.setGains(
              new AzimuthMotorGains(
                  values[0], values[1], values[2], values[3], values[4], values[5], values[6],
                  values[7]));
        },
        azimuthkP,
        azimuthkI,
        azimuthkD,
        azimuthkS,
        azimuthkV,
        azimuthkA,
        azimuthkMaxVelo,
        azimuthkMaxAccel);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state, double azimuthVelocityFF) {
    // Optimize velocity setpoint
    state = OnboardModuleState.optimize(state, getAngle());
    Logger.recordOutput(azimuthName + "/goal", state.angle.getRotations());
    state.cosineScale(Rotation2d.fromRotations(azimuthInputs.outputPositionRotations));

    driveMotor.setVelocity(
        Units.radiansToRotations(
            state.speedMetersPerSecond / DriveConstants.driveWheelRadiusMeters));

    azimuthMotor.setPosition(state.angle.getRotations(), azimuthVelocityFF);

    // if (Math.abs(azimuthSetpoint.position - azimuthGoal.position) > 0.5) {
    //   azimuthSetpoint = azimuthGoal;
    // }
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    driveMotor.setVoltage(output);
    azimuthMotor.setPosition(0.0, 0.0);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    driveMotor.setVoltage(0.0);
    azimuthMotor.setVoltage(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(azimuthInputs.outputPositionRotations);
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return Units.rotationsToRadians(driveInputs.positionRotations)
        * DriveConstants.driveWheelRadiusMeters;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return Units.rotationsToRadians(driveInputs.velocityRotationsPerSecond)
        * DriveConstants.driveWheelRadiusMeters;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return driveInputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return Units.rotationsToRadians(driveInputs.positionRotations);
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return driveInputs.positionRotations;
  }
}
