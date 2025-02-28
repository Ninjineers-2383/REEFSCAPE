package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.piece_detection.PieceDetection;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 2.0;
  private static final double ANGLE_KD = 0.0;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private static final LoggedTunableNumber PIECE_DETECTION_P =
      new LoggedTunableNumber("Drive/PieceDetection/kP", 5.0);

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          speeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation());
          drive.runVelocity(speeds);
        },
        drive);
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities),
   * being helped by a PieceDetection subsystem.
   */
  public static Command joystickDrivePieceDetection(
      Drive drive,
      PieceDetection pieceDetection,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          speeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation());

          if (Math.abs(speeds.vyMetersPerSecond) > 0.1
              && Math.abs(speeds.vyMetersPerSecond) < 0.3
              && speeds.vxMetersPerSecond < 0.1) {
            speeds.vyMetersPerSecond =
                speeds.vyMetersPerSecond
                    + new Transform3d(
                                new Pose3d(drive.getPose()), pieceDetection.getGamePiecePose())
                            .getY()
                        * PIECE_DETECTION_P.get();
          }
          drive.runVelocity(speeds);
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              speeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation());
              drive.runVelocity(speeds);
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  public static Command joystickDriveHeadingLock(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier resetHeading) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return new Command() {
      double desiredHeading = drive.getRotation().getRadians();
      boolean locked = false;

      {
        addRequirements(drive);
      }

      @Override
      public void initialize() {
        angleController.reset(drive.getRotation().getRadians());
      }

      @Override
      public void execute() {
        // Get linear velocity
        Translation2d linearVelocity =
            getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

        // Calculate angular speed
        double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

        omega = Math.copySign(omega * omega, omega) * drive.getMaxAngularSpeedRadPerSec();

        if (Math.abs(omega) == 0.0 && !resetHeading.getAsBoolean()) {
          if (!locked) {
            angleController.reset(drive.getRotation().getRadians());
            desiredHeading = drive.getRotation().getRadians();
            locked = true;
          }
          omega = angleController.calculate(drive.getRotation().getRadians(), desiredHeading);
        } else {
          locked = false;
        }

        // Convert to field relative speeds & send command
        ChassisSpeeds speeds =
            new ChassisSpeeds(
                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                omega);
        boolean isFlipped =
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
        speeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                isFlipped
                    ? drive.getRotation().plus(new Rotation2d(Math.PI))
                    : drive.getRotation());
        drive.runVelocity(speeds);
      }
    };
  }

  protected static double transformDot(Transform2d a, Transform2d b) {
    return a.getX() * b.getX() + a.getY() * b.getY();
  }

  protected static Pose2d sampleTrajectory(PathPlannerPath trajectory, Pose2d pose) {
    List<Pose2d> states = trajectory.getPathPoses();

    // Find closest point to current pose
    Pose2d lowerState = states.get(0);
    Pose2d higherState = states.get(0);
    double minDistance = Double.POSITIVE_INFINITY;
    for (int i = 0; i < states.size() - 1; i++) {
      double distance = states.get(i).getTranslation().getDistance(pose.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        lowerState = states.get(i);
        higherState = states.get(i + 1);
      }
    }

    double d = lowerState.getTranslation().getDistance(higherState.getTranslation());
    double t = (pose.getTranslation().getDistance(lowerState.getTranslation())) / d;
    t = MathUtil.clamp(t, 0.0, 1.0);

    // Interpolate between states based on distance
    // TODO: Fix this to actually get the closest point
    Pose2d retPose = lowerState.interpolate(higherState, t);
    Translation2d minusTrans = higherState.getTranslation().minus(lowerState.getTranslation());

    // Get rotation between lower and higher states
    return new Pose2d(
        retPose.getTranslation(), new Rotation2d(minusTrans.getX(), minusTrans.getY()));
  }

  public static Command driveToPose(Drive drive, Supplier<Pose2d> pose) {
    Supplier<Pose2d> pose1 =
        () -> {
          return pose.get().transformBy(new Transform2d(-0.1, 0.0, new Rotation2d()));
        };
    return Commands.sequence(new DriveToPose(drive, pose1), new DriveToPose(drive, pose));
  }

  /**
   * @param drive
   * @return
   */
  public static Command joystickDriveAlongTrajectory(
      Drive drive,
      PathPlannerPath trajectory,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      boolean invertPath) {
    // Get closest point to drivetrain along trajectory

    Pose2d end = trajectory.getPathPoses().get(trajectory.getPathPoses().size() - 1);
    Rotation2d endRotation = trajectory.getGoalEndState().rotation();

    return new FunctionalCommand(
        () -> {
          Logger.recordOutput(
              "DriveAlongTrajectory/Trajectory", trajectory.getPathPoses().toArray(new Pose2d[0]));
        },
        () -> {
          Pose2d goalState = sampleTrajectory(trajectory, drive.getPose());

          Logger.recordOutput("DriveAlongTrajectory/TrajPose", goalState);

          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          ChassisSpeeds trajectoryVelocity =
              ChassisSpeeds.fromRobotRelativeSpeeds(
                  1.0,
                  0.0,
                  0.0,
                  goalState.getRotation().rotateBy(invertPath ? Rotation2d.kPi : Rotation2d.kZero));

          ChassisSpeeds driverVelocity =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());

          driverVelocity =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  driverVelocity, isFlipped ? Rotation2d.kPi : Rotation2d.kZero);

          Transform2d Vtraj =
              new Transform2d(
                  trajectoryVelocity.vxMetersPerSecond,
                  trajectoryVelocity.vyMetersPerSecond,
                  new Rotation2d());
          Transform2d Vdrive =
              new Transform2d(
                  new Translation2d(
                      driverVelocity.vxMetersPerSecond, driverVelocity.vyMetersPerSecond),
                  new Rotation2d());
          Transform2d Vdrive_norm = Vdrive.div(Vdrive.getTranslation().getNorm());

          double dot = transformDot(Vtraj, Vdrive_norm);

          Logger.recordOutput("DriveAlongTrajectory/Dot", dot);

          Translation2d errorTranslation =
              goalState.getTranslation().minus(drive.getPose().getTranslation());

          ChassisSpeeds error =
              new ChassisSpeeds(errorTranslation.getX(), errorTranslation.getY(), 0.0).times(1);

          ChassisSpeeds speeds;

          if (dot > 0.5 && errorTranslation.getNorm() < 1) {
            speeds =
                (trajectoryVelocity.times(dot * dot * dot).times(Vdrive.getTranslation().getNorm()))
                    .plus(driverVelocity.times((1 - dot) * (1 - dot) * (1 - dot)))
                    .plus(error);

            Logger.recordOutput("DriveAlongTrajectory/Enabled", true);

          } else {
            speeds = driverVelocity;
            Logger.recordOutput("DriveAlongTrajectory/Enabled", false);
          }

          Logger.recordOutput(
              "DriveAlongTrajectory/EndGoal", new Pose2d(end.getTranslation(), endRotation));

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  new ChassisSpeeds(
                      speeds.vxMetersPerSecond,
                      speeds.vyMetersPerSecond,
                      driverVelocity.omegaRadiansPerSecond
                          - drive.getRotation().minus(endRotation).getRadians() * 1),
                  drive.getRotation()));
        },
        (interrupted) -> {},
        () -> {
          return drive.getPose().getTranslation().getDistance(end.getTranslation()) < 0.75;
        },
        drive);
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  // public static Command azimuthTuning()

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
