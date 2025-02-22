package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.QuarrelCommands;
import frc.robot.commands.QuarrelCommands.QuarrelSubsystem;
import frc.robot.commands.QuarrelPresets;
import frc.robot.commands.QuarrelPresets.QuarrelPosition;
import frc.robot.commands.position_joint.PositionJointVelocityCommand;
import frc.robot.subsystems.drive.Drive;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ReefControls {
  public static enum QUEUED_EVENT {
    NONE,
    LEFT_L1,
    LEFT_L2,
    LEFT_L3,
    LEFT_L4,
    RIGHT_L1,
    RIGHT_L2,
    RIGHT_L3,
    RIGHT_L4,
    ALGAE_HIGH,
    ALGAE_LOW
  }

  public static enum LOCATION {
    NONE,
    REEF_FRONT,
    REEF_BACK,
    REEF_FRONT_LEFT,
    REEF_FRONT_RIGHT,
    REEF_BACK_LEFT,
    REEF_BACK_RIGHT,
    HUMAN_LEFT,
    HUMAN_RIGHT
  }

  protected QuarrelSubsystem quarrelerSubsystem;
  protected Drive drive;

  protected LOCATION queuedLocation = LOCATION.NONE;
  protected LOCATION latestHumanPlayer = LOCATION.NONE;
  protected QUEUED_EVENT queuedEvent = QUEUED_EVENT.NONE;
  protected boolean driveAlongTrajDoneInt = false;
  protected Trigger driveAlongTrajDone = new Trigger(() -> driveAlongTrajDoneInt);

  protected CommandGenericHID reefControls = new CommandGenericHID(1);
  protected CommandGenericHID branchControls = new CommandGenericHID(2);

  protected Trigger reefFront = reefControls.button(Constants.OperatorButtons.LEFT.REEF_FRONT);
  protected Trigger reefBack = reefControls.button(Constants.OperatorButtons.LEFT.REEF_BACK);
  protected Trigger reefFrontLeft =
      reefControls.button(Constants.OperatorButtons.LEFT.REEF_FRONT_LEFT);
  protected Trigger reefFrontRight =
      reefControls.button(Constants.OperatorButtons.LEFT.REEF_FRONT_RIGHT);
  protected Trigger reefBackLeft =
      reefControls.button(Constants.OperatorButtons.LEFT.REEF_BACK_LEFT);
  protected Trigger reefBackRight =
      reefControls.button(Constants.OperatorButtons.LEFT.REEF_BACK_RIGHT);

  protected Trigger humanLeft = reefControls.button(Constants.OperatorButtons.LEFT.HUMAN_LEFT);
  protected Trigger humanRight = reefControls.button(Constants.OperatorButtons.LEFT.HUMAN_RIGHT);

  protected Trigger leftL1 = branchControls.button(Constants.OperatorButtons.RIGHT.LEFT_L1);
  protected Trigger leftL2 = branchControls.button(Constants.OperatorButtons.RIGHT.LEFT_L2);
  protected Trigger leftL3 = branchControls.button(Constants.OperatorButtons.RIGHT.LEFT_L3);
  protected Trigger leftL4 = branchControls.button(Constants.OperatorButtons.RIGHT.LEFT_L4);
  protected Trigger rightL1 = branchControls.button(Constants.OperatorButtons.RIGHT.RIGHT_L1);
  protected Trigger rightL2 = branchControls.button(Constants.OperatorButtons.RIGHT.RIGHT_L2);
  protected Trigger rightL3 = branchControls.button(Constants.OperatorButtons.RIGHT.RIGHT_L3);
  protected Trigger rightL4 = branchControls.button(Constants.OperatorButtons.RIGHT.RIGHT_L4);

  protected Trigger algaeHigh = branchControls.button(Constants.OperatorButtons.RIGHT.ALGAE_HIGH);
  protected Trigger algaeLow = branchControls.button(Constants.OperatorButtons.RIGHT.ALGAE_LOW);

  protected Map<QUEUED_EVENT, Command> queuedCommands = new HashMap<>();
  protected static Map<QUEUED_EVENT, Supplier<QuarrelPosition>> queuedPresets =
      new HashMap<>() {
        {
          put(QUEUED_EVENT.LEFT_L1, QuarrelPresets::getL1);
          put(QUEUED_EVENT.LEFT_L2, QuarrelPresets::getL2);
          put(QUEUED_EVENT.LEFT_L3, QuarrelPresets::getL3);
          put(QUEUED_EVENT.LEFT_L4, QuarrelPresets::getL4);
          put(QUEUED_EVENT.RIGHT_L1, QuarrelPresets::getL1);
          put(QUEUED_EVENT.RIGHT_L2, QuarrelPresets::getL2);
          put(QUEUED_EVENT.RIGHT_L3, QuarrelPresets::getL3);
          put(QUEUED_EVENT.RIGHT_L4, QuarrelPresets::getL4);
          put(QUEUED_EVENT.ALGAE_HIGH, QuarrelPresets::getHighball);
          put(QUEUED_EVENT.ALGAE_LOW, QuarrelPresets::getLowball);
        }
      };

  protected Function<PathPlannerPath, Command> driveTrajCommand;

  public ReefControls(
      QuarrelSubsystem subsystem,
      Drive drive,
      Function<PathPlannerPath, Command> driveTrajCommand) {
    this.quarrelerSubsystem = subsystem;
    this.drive = drive;
    this.driveTrajCommand = driveTrajCommand;
  }

  public void init() {
    queuedCommands.put(
        QUEUED_EVENT.LEFT_L1,
        getScoreSequence(() -> queuedLocation, QUEUED_EVENT.LEFT_L1, drive, quarrelerSubsystem));

    queuedCommands.put(
        QUEUED_EVENT.LEFT_L2,
        getScoreSequence(() -> queuedLocation, QUEUED_EVENT.LEFT_L2, drive, quarrelerSubsystem));

    queuedCommands.put(
        QUEUED_EVENT.LEFT_L3,
        getScoreSequence(() -> queuedLocation, QUEUED_EVENT.LEFT_L3, drive, quarrelerSubsystem));

    queuedCommands.put(
        QUEUED_EVENT.LEFT_L4,
        getScoreSequence(() -> queuedLocation, QUEUED_EVENT.LEFT_L4, drive, quarrelerSubsystem));

    queuedCommands.put(
        QUEUED_EVENT.RIGHT_L1,
        getScoreSequence(() -> queuedLocation, QUEUED_EVENT.RIGHT_L1, drive, quarrelerSubsystem));

    queuedCommands.put(
        QUEUED_EVENT.RIGHT_L2,
        getScoreSequence(() -> queuedLocation, QUEUED_EVENT.RIGHT_L2, drive, quarrelerSubsystem));

    queuedCommands.put(
        QUEUED_EVENT.RIGHT_L3,
        getScoreSequence(() -> queuedLocation, QUEUED_EVENT.RIGHT_L3, drive, quarrelerSubsystem));

    queuedCommands.put(
        QUEUED_EVENT.RIGHT_L4,
        getScoreSequence(() -> queuedLocation, QUEUED_EVENT.RIGHT_L4, drive, quarrelerSubsystem));

    queuedCommands.put(
        QUEUED_EVENT.ALGAE_HIGH,
        getAlgaeIntakeSequence(() -> queuedLocation, QUEUED_EVENT.ALGAE_HIGH));

    queuedCommands.put(
        QUEUED_EVENT.ALGAE_LOW,
        getAlgaeIntakeSequence(() -> queuedLocation, QUEUED_EVENT.ALGAE_LOW));

    leftL1.onTrue(
        Commands.runOnce(
            () -> {
              scoreSidePressed(queuedLocation, QUEUED_EVENT.LEFT_L1);
            }));
    leftL2.onTrue(
        Commands.runOnce(
            () -> {
              scoreSidePressed(queuedLocation, QUEUED_EVENT.LEFT_L2);
            }));
    leftL3.onTrue(
        Commands.runOnce(
            () -> {
              scoreSidePressed(queuedLocation, QUEUED_EVENT.LEFT_L3);
            }));
    leftL4.onTrue(
        Commands.runOnce(
            () -> {
              scoreSidePressed(queuedLocation, QUEUED_EVENT.LEFT_L4);
            }));
    rightL1.onTrue(
        Commands.runOnce(
            () -> {
              scoreSidePressed(queuedLocation, QUEUED_EVENT.RIGHT_L1);
            }));
    rightL2.onTrue(
        Commands.runOnce(
            () -> {
              scoreSidePressed(queuedLocation, QUEUED_EVENT.RIGHT_L2);
            }));
    rightL3.onTrue(
        Commands.runOnce(
            () -> {
              scoreSidePressed(queuedLocation, QUEUED_EVENT.RIGHT_L3);
            }));
    rightL4.onTrue(
        Commands.runOnce(
            () -> {
              scoreSidePressed(queuedLocation, QUEUED_EVENT.RIGHT_L4);
            }));

    algaeHigh.onTrue(
        Commands.runOnce(
            () -> {
              scoreSidePressed(queuedLocation, QUEUED_EVENT.ALGAE_HIGH);
            }));

    algaeLow.onTrue(
        Commands.runOnce(
            () -> {
              scoreSidePressed(queuedLocation, QUEUED_EVENT.ALGAE_LOW);
            }));

    driveAlongTrajDone.onTrue(Commands.select(queuedCommands, () -> queuedEvent));

    reefFront.onTrue(getDriveTrajCommand(LOCATION.REEF_FRONT));
    reefBack.onTrue(getDriveTrajCommand(LOCATION.REEF_BACK));
    reefFrontLeft.onTrue(getDriveTrajCommand(LOCATION.REEF_FRONT_LEFT));
    reefFrontRight.onTrue(getDriveTrajCommand(LOCATION.REEF_FRONT_RIGHT));
    reefBackLeft.onTrue(getDriveTrajCommand(LOCATION.REEF_BACK_LEFT));
    reefBackRight.onTrue(getDriveTrajCommand(LOCATION.REEF_BACK_RIGHT));

    reefControls
        .button(Constants.OperatorButtons.LEFT.EMERGENCY_ELEVATOR_RESET)
        .whileTrue(new PositionJointVelocityCommand(quarrelerSubsystem.elevator(), () -> -0.25))
        .onFalse(
            Commands.parallel(
                new PositionJointVelocityCommand(quarrelerSubsystem.elevator(), () -> 0.0),
                Commands.runOnce(() -> quarrelerSubsystem.elevator().resetPosition())));

    branchControls
        .button(Constants.OperatorButtons.RIGHT.PROCESSOR)
        .onTrue(QuarrelCommands.PresetCommand(quarrelerSubsystem, QuarrelPresets::getProcessor));

    branchControls
        .button(Constants.OperatorButtons.RIGHT.BARGE)
        .onTrue(QuarrelCommands.PresetCommand(quarrelerSubsystem, QuarrelPresets::getBargeLow));

    humanLeft.onTrue(Commands.runOnce(() -> latestHumanPlayer = LOCATION.HUMAN_LEFT));
    humanRight.onTrue(Commands.runOnce(() -> latestHumanPlayer = LOCATION.HUMAN_RIGHT));

    humanLeft.or(humanRight).onTrue(QuarrelCommands.TransferCommand(quarrelerSubsystem));
  }

  public void periodic() {

    // Update the controls
  }

  protected void setQueuedEvent(QUEUED_EVENT event) {
    queuedEvent = event;
    Logger.recordOutput("Controls/QueuedEvent", queuedEvent.toString());
  }

  protected Command getDriveTrajCommand(LOCATION reefLocation) {
    int idx = getReefSideIndex(reefLocation);
    return Commands.defer(
        () -> {
          PathPlannerPath trajectory;
          try {
            trajectory =
                PathPlannerPath.fromPathFile(
                    (latestHumanPlayer == LOCATION.HUMAN_RIGHT ? "R" : "L") + " to " + (idx + 1));
          } catch (Exception e) {
            Logger.recordOutput("Controls/DriveTrajCommand", e.toString());
            return null;
          }
          return Commands.sequence(
              Commands.runOnce(() -> driveAlongTrajDoneInt = false),
              Commands.runOnce(() -> this.queuedLocation = reefLocation),
              driveTrajCommand.apply(trajectory),
              Commands.runOnce(() -> driveAlongTrajDoneInt = true));
        },
        Set.of(drive));
  }

  public static Command getScoreSequence(
      Supplier<LOCATION> reefLocation,
      QUEUED_EVENT event,
      Drive drive,
      QuarrelSubsystem quarrelerSubsystem) {
    return Commands.sequence(
        Commands.parallel(
            DriveCommands.driveToPose(drive, () -> getScorePose(reefLocation.get(), event)),
            QuarrelCommands.PresetCommand(
                quarrelerSubsystem, () -> queuedPresets.get(event).get())),
        QuarrelCommands.ScoreCommand(quarrelerSubsystem),
        QuarrelCommands.TransferPose(quarrelerSubsystem));
  }

  public static Pose2d getScorePose(LOCATION reefLocation, QUEUED_EVENT event) {
    boolean isLeft = false;
    switch (event) {
      case LEFT_L1:
      case LEFT_L2:
      case LEFT_L3:
      case LEFT_L4:
        isLeft = true;
      default:
        break;
    }

    return REEFLocations.getInstance()
        .getBranchScorePose(getReefSideIndex(reefLocation) * 2 + (isLeft ? 0 : 1));
  }

  protected void scoreSidePressed(LOCATION reefLocation, QUEUED_EVENT event) {
    if (isCloseSide(reefLocation)) {
      setQueuedEvent(event);
    } else {
      QUEUED_EVENT flippedEvent;
      switch (event) {
        case LEFT_L1:
          flippedEvent = QUEUED_EVENT.RIGHT_L1;
          break;
        case LEFT_L2:
          flippedEvent = QUEUED_EVENT.RIGHT_L2;
          break;
        case LEFT_L3:
          flippedEvent = QUEUED_EVENT.RIGHT_L3;
          break;
        case LEFT_L4:
          flippedEvent = QUEUED_EVENT.RIGHT_L4;
          break;
        case RIGHT_L1:
          flippedEvent = QUEUED_EVENT.LEFT_L1;
          break;
        case RIGHT_L2:
          flippedEvent = QUEUED_EVENT.LEFT_L2;
          break;
        case RIGHT_L3:
          flippedEvent = QUEUED_EVENT.LEFT_L3;
          break;
        case RIGHT_L4:
          flippedEvent = QUEUED_EVENT.LEFT_L4;
          break;
        default:
          flippedEvent = event;
          break;
      }
      setQueuedEvent(flippedEvent);
    }
  }

  protected boolean isCloseSide(LOCATION reefLocation) {
    switch (reefLocation) {
      case REEF_FRONT:
      case REEF_FRONT_LEFT:
      case REEF_FRONT_RIGHT:
        return true;
      default:
        return false;
    }
  }

  public static int getReefSideIndex(LOCATION reefLocation) {
    int location;

    switch (reefLocation) {
      case REEF_FRONT:
        location = 0;
        break;
      case REEF_FRONT_RIGHT:
        location = 1;
        break;
      case REEF_BACK_RIGHT:
        location = 2;
        break;
      case REEF_BACK:
        location = 3;
        break;
      case REEF_BACK_LEFT:
        location = 4;
        break;
      case REEF_FRONT_LEFT:
        location = 5;
        break;
      default:
        location = -1;
        break;
    }

    return location;
  }

  protected Command getAlgaeIntakeSequence(Supplier<LOCATION> reefLocation, QUEUED_EVENT event) {
    return Commands.parallel(
            DriveCommands.driveToPose(
                drive,
                () ->
                    REEFLocations.getInstance()
                        .getAlgaePickupPose(getReefSideIndex(reefLocation.get()))),
            QuarrelCommands.IntakeAlgaeCommand(quarrelerSubsystem),
            QuarrelCommands.PresetCommand(quarrelerSubsystem, () -> queuedPresets.get(event).get()))
        .andThen(QuarrelCommands.HoldAlgaeCommand(quarrelerSubsystem));
  }
}
