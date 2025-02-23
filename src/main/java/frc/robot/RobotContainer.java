package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.commands.QuarrelCommands.QuarrelSubsystem;
import frc.robot.commands.QuarrelPresets;
import frc.robot.commands.QuarrelPresets.QuarrelPosition;
import frc.robot.commands.flywheel.FlywheelVoltageCommand;
import frc.robot.commands.position_joint.PositionJointPositionCommand;
import frc.robot.commands.position_joint.PositionJointVelocityCommand;
import frc.robot.subsystems.components.Components;
import frc.robot.subsystems.digital_sensor.DigitalSensor;
import frc.robot.subsystems.digital_sensor.DigitalSensorConstants;
import frc.robot.subsystems.digital_sensor.DigitalSensorIODigitalInput;
import frc.robot.subsystems.digital_sensor.DigitalSensorIOReplay;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.drive.azimuth_motor.*;
import frc.robot.subsystems.drive.drive_motor.*;
import frc.robot.subsystems.drive.gyro.*;
import frc.robot.subsystems.drive.odometry_threads.PhoenixOdometryThread;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.flywheel.FlywheelIOReplay;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.subsystems.position_joint.PositionJointConstants;
import frc.robot.subsystems.position_joint.PositionJointIOReplay;
import frc.robot.subsystems.position_joint.PositionJointIOSim;
import frc.robot.subsystems.position_joint.PositionJointIOSparkMax;
import frc.robot.subsystems.position_joint.PositionJointIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionIOPhotonVisionTrig;
import frc.robot.subsystems.vision.VisionIOQuestNav;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  private final PositionJoint elevator;
  private final PositionJoint pivot;
  private final Flywheel outtake;
  private final DigitalSensor outtake_bottom_sensor;

  private final QuarrelSubsystem quarrel;

  private final PositionJoint climber;
  private final Flywheel climberIntake;

  private final PositionJoint funnelPivot;
  private final Flywheel funnelIntake;

  @SuppressWarnings("unused")
  private final Vision vision;

  @SuppressWarnings("unused")
  private final Components sim_components;

  // Simulation

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
        VisionIOQuestNav questNav =
            new VisionIOQuestNav(
                VisionConstants.robotToCamera0,
                new VisionIOPhotonVisionTrig(
                    "OV9281-12", VisionConstants.robotToCamera1, drive::getRotation));
        // Reset gyro to 0° when B button is pressed
        driverController.b().onTrue(Commands.runOnce(questNav::resetHeading).ignoringDisable(true));

        vision = new Vision(drive::addVisionMeasurement, questNav);

        elevator =
            new PositionJoint(
                new PositionJointIOTalonFX("Elevator", PositionJointConstants.ELEVATOR_CONFIG),
                PositionJointConstants.ELEVATOR_GAINS);

        pivot =
            new PositionJoint(
                new PositionJointIOTalonFX("Pivot", PositionJointConstants.PIVOT_CONFIG),
                PositionJointConstants.PIVOT_GAINS);

        outtake =
            new Flywheel(
                new FlywheelIOTalonFX("Outtake", FlywheelConstants.OUTTAKE_CONFIG),
                FlywheelConstants.OUTTAKE_GAINS);

        outtake_bottom_sensor =
            new DigitalSensor(
                new DigitalSensorIODigitalInput(
                    "Outtake_Bottom_Sensor", DigitalSensorConstants.OUTTAKE_BOTTOM_BREAK_CONFIG));

        climber =
            new PositionJoint(
                new PositionJointIOTalonFX("Climber", PositionJointConstants.CLIMBER_CONFIG),
                PositionJointConstants.CLIMBER_GAINS);

        climberIntake =
            new Flywheel(
                new FlywheelIOTalonFX("Climber_Intake", FlywheelConstants.CLIMBER_INTAKE_CONFIG),
                FlywheelConstants.CLIMBER_INTAKE_GAINS);

        funnelPivot =
            new PositionJoint(
                new PositionJointIOSparkMax(
                    "FunnelPivot", PositionJointConstants.FUNNEL_PIVOT_CONFIG, () -> 0, false),
                PositionJointConstants.FUNNEL_PIVOT_GAINS);

        funnelIntake =
            new Flywheel(
                new FlywheelIOSparkMax(
                    "FunnelIntake", FlywheelConstants.FUNNEL_INTAKE_CONFIG, false),
                FlywheelConstants.FUNNEL_INTAKE_GAINS);
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

        elevator =
            new PositionJoint(
                new PositionJointIOSim("Elevator", PositionJointConstants.ELEVATOR_CONFIG),
                PositionJointConstants.ELEVATOR_GAINS);

        pivot =
            new PositionJoint(
                new PositionJointIOSim("Pivot", PositionJointConstants.PIVOT_CONFIG),
                PositionJointConstants.PIVOT_GAINS);

        outtake =
            new Flywheel(
                new FlywheelIOSim("Outtake", FlywheelConstants.OUTTAKE_CONFIG),
                FlywheelConstants.OUTTAKE_GAINS);

        outtake_bottom_sensor =
            new DigitalSensor(new DigitalSensorIOReplay("Outtake_Bottom_Sensor"));

        climber =
            new PositionJoint(
                new PositionJointIOSim("Climber", PositionJointConstants.CLIMBER_CONFIG),
                PositionJointConstants.CLIMBER_GAINS);

        climberIntake =
            new Flywheel(
                new FlywheelIOSim("Climber_Intake", FlywheelConstants.CLIMBER_INTAKE_CONFIG),
                FlywheelConstants.CLIMBER_INTAKE_GAINS);

        funnelPivot =
            new PositionJoint(
                new PositionJointIOSim("FunnelPivot", PositionJointConstants.FUNNEL_PIVOT_CONFIG),
                PositionJointConstants.FUNNEL_PIVOT_GAINS);

        funnelIntake =
            new Flywheel(
                new FlywheelIOSim("FunnelIntake", FlywheelConstants.FUNNEL_INTAKE_CONFIG),
                FlywheelConstants.FUNNEL_INTAKE_GAINS);

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

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});

        pivot =
            new PositionJoint(
                new PositionJointIOReplay("Pivot"), PositionJointConstants.PIVOT_GAINS);
        elevator =
            new PositionJoint(
                new PositionJointIOReplay("Elevator"), PositionJointConstants.ELEVATOR_GAINS);

        outtake = new Flywheel(new FlywheelIOReplay("Outtake"), FlywheelConstants.OUTTAKE_GAINS);

        outtake_bottom_sensor =
            new DigitalSensor(new DigitalSensorIOReplay("Outtake_Bottom_Sensor"));

        climber =
            new PositionJoint(
                new PositionJointIOReplay("Climber"), PositionJointConstants.CLIMBER_GAINS);

        climberIntake =
            new Flywheel(
                new FlywheelIOReplay("Climber_Intake"), FlywheelConstants.CLIMBER_INTAKE_GAINS);

        funnelPivot =
            new PositionJoint(
                new PositionJointIOReplay("FunnelPivot"),
                PositionJointConstants.FUNNEL_PIVOT_GAINS);

        funnelIntake =
            new Flywheel(
                new FlywheelIOReplay("FunnelIntake"), FlywheelConstants.FUNNEL_INTAKE_GAINS);
        break;
    }

    sim_components = new Components(elevator);

    quarrel =
        new QuarrelSubsystem(
            elevator, pivot, outtake, funnelPivot, funnelIntake, outtake_bottom_sensor);

    L1Chooser = new LoggedNetworkBoolean("/Coral Choosers/L1", false);
    L2Chooser = new LoggedNetworkBoolean("/Coral Choosers/L2", false);
    L3Chooser = new LoggedNetworkBoolean("/Coral Choosers/L3", false);
    L4Chooser = new LoggedNetworkBoolean("/Coral Choosers/L4", false);
    ZeroChooser = new LoggedNetworkBoolean("/Coral Choosers/Zero", false);
    ScoreChooser = new LoggedNetworkBoolean("/Coral Choosers/Score", false);
    LowballChooser = new LoggedNetworkBoolean("/Coral Choosers/Lowball", false);
    HighballChooser = new LoggedNetworkBoolean("/Coral Choosers/Highball", false);

    TransferChooser = new LoggedNetworkBoolean("/Coral Choosers/Transfer", false);

    // Configure the button bindings
    configureButtonBindings();

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

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    new ReefControls(
            quarrel,
            drive,
            (path) ->
                DriveCommands.joystickDriveAlongTrajectory(
                    drive,
                    path.flipPath(),
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    () -> -driverController.getRightX(),
                    false))
        .init();

    driverController
        .a()
        .onTrue(
            Commands.parallel(
                QuarrelCommands.PresetCommand(quarrel, QuarrelPresets::getBargeHigh),
                new FlywheelVoltageCommand(outtake, () -> 4)));

    driverController
        .x()
        .onTrue(
            Commands.parallel(
                QuarrelCommands.PresetCommand(
                    quarrel, () -> new QuarrelPosition(0, Rotation2d.kZero)),
                new PositionJointPositionCommand(funnelPivot, () -> 0.4),
                new FlywheelVoltageCommand(climberIntake, () -> 12.0).withTimeout(0.2),
                new PositionJointPositionCommand(climber, () -> 290)));

    // outtake.setDefaultCommand(new FlywheelVoltageCommand(outtake, () -> 0));
    funnelIntake.setDefaultCommand(new FlywheelVoltageCommand(funnelIntake, () -> 0));

    driverController
        .rightBumper()
        .onTrue(new FlywheelVoltageCommand(outtake, () -> 3.0))
        .onFalse(new FlywheelVoltageCommand(outtake, () -> 0.0));
    driverController
        .leftBumper()
        .onTrue(new FlywheelVoltageCommand(outtake, () -> -3.0))
        .onFalse(new FlywheelVoltageCommand(outtake, () -> 0.0));

    climber.setDefaultCommand(
        new PositionJointVelocityCommand(
            climber,
            () ->
                (driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis())
                    * 2.0));

    driverController
        .pov(0)
        .onTrue(
            Commands.parallel(
                new FlywheelVoltageCommand(climberIntake, () -> -12.0).withTimeout(0.2),
                new PositionJointPositionCommand(climber, () -> 225),
                new PositionJointPositionCommand(pivot, () -> 0),
                new PositionJointPositionCommand(funnelPivot, () -> 0.4)));

    driverController
        .pov(180)
        .onTrue(
            Commands.parallel(
                new FlywheelVoltageCommand(climberIntake, () -> 0.0).withTimeout(0.2),
                new PositionJointPositionCommand(climber, () -> 100)));

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

                  if (elevator.getCurrentCommand() != null) {
                    elevator.getCurrentCommand().cancel();
                  }
                }));

    new Trigger(L1Chooser::get)
        .onTrue(QuarrelCommands.PresetCommand(quarrel, QuarrelPresets::getL1));

    new Trigger(L2Chooser::get)
        .onTrue(QuarrelCommands.PresetCommand(quarrel, QuarrelPresets::getL2));

    new Trigger(L3Chooser::get)
        .onTrue(QuarrelCommands.PresetCommand(quarrel, QuarrelPresets::getL3));

    new Trigger(L4Chooser::get)
        .onTrue(
            Commands.parallel(
                Commands.sequence(
                    Commands.waitUntil(() -> elevator.getPosition() > 0.6),
                    new PositionJointPositionCommand(
                        pivot, () -> QuarrelPresets.getL4().pivotRotation().getRotations())),
                new PositionJointPositionCommand(
                    elevator, () -> QuarrelPresets.getL4().elevatorPositionMeters())));

    new Trigger(ZeroChooser::get)
        .onTrue(QuarrelCommands.PresetCommand(quarrel, QuarrelPresets::getZero));

    new Trigger(ScoreChooser::get).onTrue(QuarrelCommands.ScoreCommand(quarrel));

    // new Trigger(HighballChooser::get)
    //     .onTrue(
    //         QuarrelCommands.E(
    //             elevator, outtake, QuarrelPresets::getHighball, () -> -12.0));

    // new Trigger(LowballChooser::get)
    //     .onTrue(
    //         QuarrelCommands.QuarrelCommand(
    //             elevator, outtake, QuarrelPresets::getLowball, () -> -12.0));

    new Trigger(TransferChooser::get).onTrue(QuarrelCommands.TransferCommand(quarrel));

    // buttonBoard1
    //     .button(1)
    //     .onTrue(
    //         AutoBuilder.pathfindToPose(
    //             new Pose2d(
    //                 13.890498 + Units.inchesToMeters(27 / 2.0 + 3.0),
    //                 4.0259 + Units.inchesToMeters(3),
    //                 new Rotation2d(Math.PI)),
    //             PathConstraints.unlimitedConstraints(12)));

    NamedCommands.registerCommand(
        "L4_5",
        Commands.sequence(
            ReefControls.getScoreSequence(
                () -> {
                  return ReefControls.LOCATION.REEF_BACK_LEFT;
                },
                ReefControls.QUEUED_EVENT.LEFT_L4,
                drive,
                quarrel)));
    NamedCommands.registerCommand(
        "L4_5",
        Commands.sequence(
            ReefControls.getScoreSequence(
                () -> {
                  return ReefControls.LOCATION.REEF_BACK_LEFT;
                },
                ReefControls.QUEUED_EVENT.LEFT_L4,
                drive,
                quarrel)));
    NamedCommands.registerCommand(
        "L4_6",
        Commands.sequence(
            ReefControls.getScoreSequence(
                () -> {
                  return ReefControls.LOCATION.REEF_FRONT_LEFT;
                },
                ReefControls.QUEUED_EVENT.LEFT_L4,
                drive,
                quarrel)));

    NamedCommands.registerCommand(
        "L4_6R",
        Commands.sequence(
            ReefControls.getScoreSequence(
                () -> {
                  return ReefControls.LOCATION.REEF_FRONT_RIGHT;
                },
                ReefControls.QUEUED_EVENT.RIGHT_L4,
                drive,
                quarrel)));

    NamedCommands.registerCommand(
        "L4_3",
        Commands.sequence(
            ReefControls.getScoreSequence(
                () -> {
                  return ReefControls.LOCATION.REEF_BACK_RIGHT;
                },
                ReefControls.QUEUED_EVENT.LEFT_L4,
                drive,
                quarrel)));
    NamedCommands.registerCommand(
        "L4_2",
        Commands.sequence(
            ReefControls.getScoreSequence(
                () -> {
                  return ReefControls.LOCATION.REEF_FRONT_RIGHT;
                },
                ReefControls.QUEUED_EVENT.LEFT_L4,
                drive,
                quarrel)));
    NamedCommands.registerCommand(
        "L4_2R",
        Commands.sequence(
            ReefControls.getScoreSequence(
                () -> {
                  return ReefControls.LOCATION.REEF_FRONT_RIGHT;
                },
                ReefControls.QUEUED_EVENT.RIGHT_L4,
                drive,
                quarrel)));

    NamedCommands.registerCommand(
        "MidPointReady",
        Commands.sequence(QuarrelCommands.PresetCommand(quarrel, QuarrelPresets::getMID)));

    NamedCommands.registerCommand("Score", QuarrelCommands.ScoreCommand(quarrel));
    NamedCommands.registerCommand("Transfer", QuarrelCommands.TransferCommand(quarrel));

    NamedCommands.registerCommand(
        "TransferUP",
        Commands.sequence(
            QuarrelCommands.TransferCommand(quarrel),
            QuarrelCommands.PresetCommand(quarrel, QuarrelPresets::getMID)));
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
