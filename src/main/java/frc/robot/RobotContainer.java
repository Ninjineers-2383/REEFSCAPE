package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorConstants;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIOReplay;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIOSim;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIOTalonFX;
import frc.robot.subsystems.drive.drive_motor.DriveMotorConstants;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIOReplay;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIOSim;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIOTalonFX;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.odometry_threads.PhoenixOdometryThread;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  @SuppressWarnings("unused")
  private final Vision vision;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations

        // If using REV hardware, set up the Spark Odometry Thread, if using CTRE hardware, set up
        // the Phoenix Odometry Thread, if using a combination of the two, set up both
        drive =
            new Drive(
                new GyroIOPigeon2(0, "Drive"),
                new Module(
                    new DriveMotorIOTalonFX(
                        "FrontLeftDrive", DriveMotorConstants.FRONT_LEFT_CONFIG),
                    DriveMotorConstants.FRONT_LEFT_GAINS,
                    new AzimuthMotorIOTalonFX(
                        "FrontLeftAz", AzimuthMotorConstants.FRONT_LEFT_CONFIG),
                    AzimuthMotorConstants.FRONT_LEFT_GAINS),
                new Module(
                    new DriveMotorIOTalonFX(
                        "FrontRightDrive", DriveMotorConstants.FRONT_RIGHT_CONFIG),
                    DriveMotorConstants.FRONT_RIGHT_GAINS,
                    new AzimuthMotorIOTalonFX(
                        "FrontRightAz", AzimuthMotorConstants.FRONT_RIGHT_CONFIG),
                    AzimuthMotorConstants.FRONT_RIGHT_GAINS),
                new Module(
                    new DriveMotorIOTalonFX("BackLeftDrive", DriveMotorConstants.BACK_LEFT_CONFIG),
                    DriveMotorConstants.BACK_LEFT_GAINS,
                    new AzimuthMotorIOTalonFX("BackLeftAz", AzimuthMotorConstants.BACK_LEFT_CONFIG),
                    AzimuthMotorConstants.BACK_LEFT_GAINS),
                new Module(
                    new DriveMotorIOTalonFX(
                        "BackRightDrive", DriveMotorConstants.BACK_RIGHT_CONFIG),
                    DriveMotorConstants.BACK_RIGHT_GAINS,
                    new AzimuthMotorIOTalonFX(
                        "BackRightAz", AzimuthMotorConstants.BACK_RIGHT_CONFIG),
                    AzimuthMotorConstants.BACK_RIGHT_GAINS),
                PhoenixOdometryThread.getInstance(),
                null);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0));
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new Module(
                    new DriveMotorIOSim("FrontLeftDrive", DriveMotorConstants.FRONT_LEFT_CONFIG),
                    DriveMotorConstants.FRONT_LEFT_GAINS,
                    new AzimuthMotorIOSim("FrontLeftAz", AzimuthMotorConstants.FRONT_LEFT_CONFIG),
                    AzimuthMotorConstants.FRONT_LEFT_GAINS),
                new Module(
                    new DriveMotorIOSim("FrontRightDrive", DriveMotorConstants.FRONT_RIGHT_CONFIG),
                    DriveMotorConstants.FRONT_RIGHT_GAINS,
                    new AzimuthMotorIOSim("FrontRightAz", AzimuthMotorConstants.FRONT_RIGHT_CONFIG),
                    AzimuthMotorConstants.FRONT_RIGHT_GAINS),
                new Module(
                    new DriveMotorIOSim("BackLeftDrive", DriveMotorConstants.BACK_LEFT_CONFIG),
                    DriveMotorConstants.BACK_LEFT_GAINS,
                    new AzimuthMotorIOSim("BackLeftAz", AzimuthMotorConstants.BACK_LEFT_CONFIG),
                    AzimuthMotorConstants.BACK_LEFT_GAINS),
                new Module(
                    new DriveMotorIOSim("BackRightDrive", DriveMotorConstants.BACK_RIGHT_CONFIG),
                    DriveMotorConstants.BACK_RIGHT_GAINS,
                    new AzimuthMotorIOSim("BackRightAz", AzimuthMotorConstants.BACK_RIGHT_CONFIG),
                    AzimuthMotorConstants.BACK_RIGHT_GAINS),
                null,
                null);

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new Module(
                    new DriveMotorIOReplay("FrontLeftDrive"),
                    DriveMotorConstants.FRONT_LEFT_GAINS,
                    new AzimuthMotorIOReplay("FrontLeftAz"),
                    AzimuthMotorConstants.FRONT_LEFT_GAINS),
                new Module(
                    new DriveMotorIOReplay("FrontRightDrive"),
                    DriveMotorConstants.FRONT_RIGHT_GAINS,
                    new AzimuthMotorIOReplay("FrontRightAz"),
                    AzimuthMotorConstants.FRONT_RIGHT_GAINS),
                new Module(
                    new DriveMotorIOReplay("BackLeftDrive"),
                    DriveMotorConstants.BACK_LEFT_GAINS,
                    new AzimuthMotorIOReplay("BackLeftAz"),
                    AzimuthMotorConstants.BACK_LEFT_GAINS),
                new Module(
                    new DriveMotorIOReplay("BackRightDrive"),
                    DriveMotorConstants.BACK_RIGHT_GAINS,
                    new AzimuthMotorIOReplay("BackRightAz"),
                    AzimuthMotorConstants.BACK_RIGHT_GAINS),
                null,
                null);
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Lock to 0° when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // // Reset gyro / odometry
    final Runnable resetGyro =
        () ->
            drive.setPose(
                new Pose2d(
                    drive.getPose().getTranslation(),
                    DriverStation.getAlliance().isPresent()
                        ? (DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                            ? new Rotation2d(Math.PI)
                            : new Rotation2d())
                        : new Rotation2d())); // zero gyro

    // Reset gyro to 0° when B button is pressed
    driverController.b().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
