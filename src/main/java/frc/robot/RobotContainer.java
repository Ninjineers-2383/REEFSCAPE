package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.QuarrelCommands;
import frc.robot.commands.QuarrelPresets;
import frc.robot.commands.flywheel.FlywheelVoltageCommand;
import frc.robot.subsystems.components.Components;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.spark.ModuleIOSparkSim;
import frc.robot.subsystems.drive.talon.ModuleIOTalonFX;
import frc.robot.subsystems.drive.talon.PhoenixOdometryThread;
import frc.robot.subsystems.drive.talon.TalonFXModuleConstants;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.subsystems.position_joint.PositionJointConstants;
import frc.robot.subsystems.position_joint.PositionJointIO;
import frc.robot.subsystems.position_joint.PositionJointIOSim;
import frc.robot.subsystems.position_joint.PositionJointIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  private final PositionJoint elevator;
  private final PositionJoint pivot;
  private final Flywheel claw;

  private final PositionJoint climber;
  private final Flywheel climberIntake;

  private final Flywheel intake;

  private final Vision vision;

  private final Components sim_components;

  // Simulation
  private SwerveDriveSimulation driveSimulation = null;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final LoggedNetworkBoolean L1Chooser;
  private final LoggedNetworkBoolean L2Chooser;
  private final LoggedNetworkBoolean L3Chooser;
  private final LoggedNetworkBoolean L4Chooser;
  private final LoggedNetworkBoolean ZeroChooser;
  private final LoggedNetworkBoolean LowballChooser;
  private final LoggedNetworkBoolean HighballChooser;

  private final LoggedNetworkBoolean ScoreChooser;

  private final LoggedNetworkBoolean TransferChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    new QuarrelPresets();
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

        elevator =
            new PositionJoint(
                new PositionJointIOTalonFX("Elevator", PositionJointConstants.ELEVATOR_CONFIG),
                PositionJointConstants.ELEVATOR_GAINS);

        pivot =
            new PositionJoint(
                new PositionJointIOTalonFX("Pivot", PositionJointConstants.PIVOT_CONFIG),
                PositionJointConstants.PIVOT_GAINS);

        claw =
            new Flywheel(
                new FlywheelIOTalonFX("Claw", FlywheelConstants.CLAW_CONFIG),
                FlywheelConstants.CLAW_GAINS);

        climber =
            new PositionJoint(
                new PositionJointIOTalonFX("Climber", PositionJointConstants.CLIMBER_CONFIG),
                PositionJointConstants.CLIMBER_GAINS);

        climberIntake =
            new Flywheel(
                new FlywheelIOTalonFX("Climber_Intake", FlywheelConstants.CLIMBER_INTAKE_CONFIG),
                FlywheelConstants.CLIMBER_INTAKE_GAINS);

        intake =
            new Flywheel(
                new FlywheelIOTalonFX("Intake", FlywheelConstants.INTAKE_CONFIG),
                FlywheelConstants.INTAKE_GAINS);
        break;

      case SIM:
        // create a maple-sim swerve drive simulation instance
        driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(7, 5.5, new Rotation2d()));
        // add the simulated drivetrain to the simulation field
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSparkSim(driveSimulation.getModules()[0]),
                new ModuleIOSparkSim(driveSimulation.getModules()[1]),
                new ModuleIOSparkSim(driveSimulation.getModules()[2]),
                new ModuleIOSparkSim(driveSimulation.getModules()[3]),
                null);

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name,
                    VisionConstants.robotToCamera0,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name,
                    VisionConstants.robotToCamera1,
                    driveSimulation::getSimulatedDriveTrainPose));

        elevator =
            new PositionJoint(
                new PositionJointIOSim("Elevator", PositionJointConstants.ELEVATOR_CONFIG),
                PositionJointConstants.ELEVATOR_GAINS);

        pivot =
            new PositionJoint(
                new PositionJointIOSim("Pivot", PositionJointConstants.PIVOT_CONFIG),
                PositionJointConstants.PIVOT_GAINS);

        claw =
            new Flywheel(
                new FlywheelIOSim("Claw", FlywheelConstants.CLAW_CONFIG),
                FlywheelConstants.CLAW_GAINS);

        climber =
            new PositionJoint(
                new PositionJointIOSim("Climber", PositionJointConstants.CLIMBER_CONFIG),
                PositionJointConstants.CLIMBER_GAINS);

        climberIntake =
            new Flywheel(
                new FlywheelIOSim("Climber_Intake", FlywheelConstants.CLIMBER_INTAKE_CONFIG),
                FlywheelConstants.CLIMBER_INTAKE_GAINS);

        intake =
            new Flywheel(
                new FlywheelIOSim("Intake", FlywheelConstants.INTAKE_CONFIG),
                FlywheelConstants.INTAKE_GAINS);
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

        elevator =
            new PositionJoint(new PositionJointIO() {}, PositionJointConstants.ELEVATOR_GAINS);

        pivot = new PositionJoint(new PositionJointIO() {}, PositionJointConstants.PIVOT_GAINS);

        claw = new Flywheel(new FlywheelIO() {}, FlywheelConstants.CLAW_GAINS);

        climber = new PositionJoint(new PositionJointIO() {}, PositionJointConstants.CLIMBER_GAINS);

        climberIntake = new Flywheel(new FlywheelIO() {}, FlywheelConstants.CLIMBER_INTAKE_GAINS);

        intake = new Flywheel(new FlywheelIO() {}, FlywheelConstants.INTAKE_GAINS);

        break;
    }

    sim_components = new Components(elevator, pivot);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    L1Chooser = new LoggedNetworkBoolean("/Coral Choosers/L1", false);
    L2Chooser = new LoggedNetworkBoolean("/Coral Choosers/L2", false);
    L3Chooser = new LoggedNetworkBoolean("/Coral Choosers/L3", false);
    L4Chooser = new LoggedNetworkBoolean("/Coral Choosers/L4", false);
    ZeroChooser = new LoggedNetworkBoolean("/Coral Choosers/Zero", false);
    ScoreChooser = new LoggedNetworkBoolean("/Coral Choosers/Score", false);
    LowballChooser = new LoggedNetworkBoolean("/Coral Choosers/Lowball", false);
    HighballChooser = new LoggedNetworkBoolean("/Coral Choosers/Highball", false);

    TransferChooser = new LoggedNetworkBoolean("/Coral Choosers/Transfer", false);
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

    Logger.recordOutput("Components", new Pose3d[] {new Pose3d()});
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    try {

      drive.setDefaultCommand(
          DriveCommands.joystickDriveAlongTrajectory(
              drive,
              TrajectoryUtil.fromPathweaverJson(
                  Filesystem.getDeployDirectory().toPath().resolve("paths/TestPath1.wpilib.json")),
              () -> -driverController.getLeftY(),
              () -> -driverController.getLeftX(),
              () -> -driverController.getRightX()));

    } catch (Exception e) {
      e.printStackTrace();
    }
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

    // Reset gyro / odometry
    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () ->
                drive.setPose(
                    driveSimulation
                        .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
            // simulation
            : () ->
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

    driverController
        .rightBumper()
        .onTrue(new FlywheelVoltageCommand(claw, () -> 3.0))
        .onFalse(new FlywheelVoltageCommand(claw, () -> 0.0));
    driverController
        .leftBumper()
        .onTrue(new FlywheelVoltageCommand(claw, () -> -3.0))
        .onFalse(new FlywheelVoltageCommand(claw, () -> 0.0));

    intake.setDefaultCommand(
        new FlywheelVoltageCommand(
            intake,
            () ->
                (driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis())
                    * 12.0));

    new Trigger(L1Chooser::get)
        .or(L2Chooser::get)
        .or(L3Chooser::get)
        .or(L4Chooser::get)
        .or(ZeroChooser::get)
        .or(ScoreChooser::get)
        .or(HighballChooser::get)
        .or(LowballChooser::get)
        .or(TransferChooser::get)
        .onTrue(
            new InstantCommand(
                () -> {
                  L1Chooser.set(false);
                  L2Chooser.set(false);
                  L3Chooser.set(false);
                  L4Chooser.set(false);
                  ZeroChooser.set(false);
                  ScoreChooser.set(false);
                  HighballChooser.set(false);
                  LowballChooser.set(false);
                  TransferChooser.set(false);

                  if (pivot.getCurrentCommand() != null) {
                    pivot.getCurrentCommand().cancel();
                  }

                  if (elevator.getCurrentCommand() != null) {
                    elevator.getCurrentCommand().cancel();
                  }
                }));

    new Trigger(L1Chooser::get)
        .onTrue(
            QuarrelCommands.QuarrelCommand(
                elevator, pivot, claw, QuarrelPresets::getL1, () -> 0.0));

    new Trigger(L2Chooser::get)
        .onTrue(
            QuarrelCommands.QuarrelCommand(
                elevator, pivot, claw, QuarrelPresets::getL2, () -> 0.0));

    new Trigger(L3Chooser::get)
        .onTrue(
            QuarrelCommands.QuarrelCommand(
                elevator, pivot, claw, QuarrelPresets::getL3, () -> 0.0));

    new Trigger(L4Chooser::get)
        .onTrue(
            QuarrelCommands.QuarrelCommand(
                elevator, pivot, claw, QuarrelPresets::getL4, () -> 0.0));

    new Trigger(ZeroChooser::get)
        .onTrue(
            QuarrelCommands.QuarrelCommand(
                elevator, pivot, claw, QuarrelPresets::getZero, () -> 0.0));

    new Trigger(ScoreChooser::get).onTrue(QuarrelCommands.ScoreCommand(elevator, pivot, claw));

    new Trigger(HighballChooser::get)
        .onTrue(
            QuarrelCommands.QuarrelCommand(
                elevator, pivot, claw, QuarrelPresets::getHighball, () -> -12.0));

    new Trigger(LowballChooser::get)
        .onTrue(
            QuarrelCommands.QuarrelCommand(
                elevator, pivot, claw, QuarrelPresets::getLowball, () -> -12.0));

    new Trigger(TransferChooser::get)
        .onTrue(QuarrelCommands.TransferCommand(elevator, pivot, claw));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(drive.getPose());
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Notes",
        SimulatedArena.getInstance().getGamePiecesByType("Note").toArray(new Pose3d[0]));
  }
}
