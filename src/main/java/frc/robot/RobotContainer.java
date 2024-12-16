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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedConditions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.flywheel.FlywheelVelocityCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.talon.ModuleIOTalonFX;
import frc.robot.subsystems.drive.talon.PhoenixOdometryThread;
import frc.robot.subsystems.drive.talon.TalonFXModuleConstants;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
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
  private final Flywheel leftShooter;
  private final Flywheel rightShooter;
  private final Flywheel intake;

  @SuppressWarnings("unused")
  private final Vision vision;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(0),
                new ModuleIOTalonFX(TalonFXModuleConstants.frontLeft),
                new ModuleIOTalonFX(TalonFXModuleConstants.frontRight),
                new ModuleIOTalonFX(TalonFXModuleConstants.rearLeft),
                new ModuleIOTalonFX(TalonFXModuleConstants.rearRight),
                PhoenixOdometryThread.getInstance());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0));
        leftShooter =
            new Flywheel(
                new FlywheelIOTalonFX("LeftShooter", FlywheelConstants.left_shooter),
                FlywheelConstants.left_shooter_gains);
        rightShooter =
            new Flywheel(
                new FlywheelIOTalonFX("RightShooter", FlywheelConstants.right_shooter),
                FlywheelConstants.right_shooter_gains);
        intake =
            new Flywheel(
                new FlywheelIOTalonFX("Intake", FlywheelConstants.intake),
                FlywheelConstants.intake_gains);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                null);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));

        leftShooter = new Flywheel(new FlywheelIO() {}, FlywheelConstants.left_shooter_gains);
        rightShooter = new Flywheel(new FlywheelIO() {}, FlywheelConstants.right_shooter_gains);
        intake = new Flywheel(new FlywheelIO() {}, FlywheelConstants.intake_gains);

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                null);
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        leftShooter = new Flywheel(new FlywheelIO() {}, FlywheelConstants.left_shooter_gains);
        rightShooter = new Flywheel(new FlywheelIO() {}, FlywheelConstants.right_shooter_gains);
        intake = new Flywheel(new FlywheelIO() {}, FlywheelConstants.intake_gains);
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    NamedConditions.registerCondition(
        "HasGamePiece", () -> SmartDashboard.getBoolean("HasGamePiece", false));
    NamedConditions.registerCondition(
        "GoTo1Or2", () -> SmartDashboard.getBoolean("GoTo1Or2", false));

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

    autoChooser.addOption("Example Auto", AutoBuilder.buildAuto("Example Auto"));
    autoChooser.addOption("Subwoofer Start", AutoBuilder.buildAuto("Subwoofer Start"));

    SmartDashboard.putBoolean("HasGamePiece", false);
    SmartDashboard.putBoolean("Continue", false);

    // Configure the button bindings
    configureButtonBindings();

    leftShooter.setDefaultCommand(
        new FlywheelVelocityCommand(
            leftShooter, () -> MathUtil.applyDeadband(operatorController.getLeftY(), 0.1) * 100));
    rightShooter.setDefaultCommand(
        new FlywheelVelocityCommand(
            rightShooter, () -> MathUtil.applyDeadband(operatorController.getRightY(), 0.1) * 100));
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

    // Reset gyro to 0° when B button is pressed
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                DriverStation.getAlliance().isPresent()
                                    ? (DriverStation.getAlliance().get()
                                            == DriverStation.Alliance.Red
                                        ? new Rotation2d(Math.PI)
                                        : new Rotation2d())
                                    : new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    operatorController
        .a()
        .onTrue(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new FlywheelVelocityCommand(leftShooter, () -> 120),
                    new FlywheelVelocityCommand(rightShooter, () -> 75)),
                new PrintCommand("Shooting"),
                new WaitCommand(2),
                new ParallelCommandGroup(
                    new FlywheelVelocityCommand(leftShooter, () -> 0),
                    new FlywheelVelocityCommand(rightShooter, () -> 0))));
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
