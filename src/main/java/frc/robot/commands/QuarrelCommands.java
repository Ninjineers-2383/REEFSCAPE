package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.QuarrelPresets.QuarrelPosition;
import frc.robot.commands.flywheel.FlywheelVoltageCommand;
import frc.robot.commands.position_joint.PositionJointPositionCommand;
import frc.robot.subsystems.digital_sensor.DigitalSensor;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;
import java.util.function.Supplier;

public class QuarrelCommands {
  private QuarrelCommands() {}

  public static record QuarrelSubsystem(
      PositionJoint elevator,
      PositionJoint pivot,
      Flywheel claw,
      PositionJoint funnelPivot,
      Flywheel funnel,
      DigitalSensor bottomBeamBreak) {}

  //   Commands.sequence(
  // Commands.waitUntil(() -> elevator.getPosition() > 0.6),
  // new PositionJointPositionCommand(
  //     pivot, () -> QuarrelPresets.getL4().pivotRotation().getRotations())),
  public static Command PresetCommand(
      QuarrelSubsystem subsystem, Supplier<QuarrelPosition> position) {
    return Commands.parallel(
        Commands.sequence(
            Commands.either(
                Commands.waitUntil(() -> subsystem.elevator.getPosition() > 0.6),
                Commands.none(),
                () -> position.get().pivotRotation().getDegrees() > 98),
            new PositionJointPositionCommand(
                subsystem.pivot, () -> position.get().pivotRotation().getRotations())),
        new PositionJointPositionCommand(
            subsystem.elevator, () -> position.get().elevatorPositionMeters()));
  }

  public static Command ScoreCommand(QuarrelSubsystem subsystem) {
    // Score by running claw at full speed until beam breaks are no longer triggered
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new WaitUntilCommand(
                subsystem
                    .bottomBeamBreak
                    .getTrigger()
                    .or(subsystem.bottomBeamBreak.getTrigger())
                    .negate()),
            new FlywheelVoltageCommand(
                subsystem.claw, () -> subsystem.pivot.getPosition() > 0.5 ? -10 : 10.0)),
        new WaitCommand(0.5),
        new FlywheelVoltageCommand(subsystem.claw, () -> 0.0).withTimeout(0.02));
  }

  public static Command TransferPose(QuarrelSubsystem subsystem) {
    return Commands.sequence(
        new PrintCommand("Transfer Command Started"),
        new PositionJointPositionCommand(
            subsystem.pivot, () -> QuarrelPresets.getTransferDown().pivotRotation().getRotations()),
        new PrintCommand("Transfer Command Pivot Finished"),
        new PositionJointPositionCommand(
            subsystem.elevator, () -> QuarrelPresets.getTransferDown().elevatorPositionMeters()),
        new PrintCommand("Transfer Command Move Finished"));
  }

  public static Command TransferCommand(QuarrelSubsystem subsystem) {
    Command transfer =
        new SequentialCommandGroup(
            Commands.parallel(
                new FlywheelVoltageCommand(subsystem.claw, () -> 5.0).withTimeout(0.2),
                new FlywheelVoltageCommand(subsystem.funnel, () -> 8.0).withTimeout(0.2)),
            new WaitUntilCommand(subsystem.bottomBeamBreak.getTrigger()),
            new WaitCommand(0.2),
            Commands.parallel(
                new FlywheelVoltageCommand(subsystem.claw, () -> 0.0).withTimeout(0.2),
                new FlywheelVoltageCommand(subsystem.funnel, () -> 0.0).withTimeout(0.2)),
            new PrintCommand("Transfer Command Index Finished"));

    return Commands.sequence(
        TransferPose(subsystem),
        Commands.either(Commands.none(), transfer, subsystem.bottomBeamBreak.getTrigger()));
  }

  public static Command IntakeAlgaeCommand(QuarrelSubsystem subsystem) {
    return new FlywheelVoltageCommand(subsystem.claw, () -> -3.0).withTimeout(0.2);
  }

  public static Command HoldAlgaeCommand(QuarrelSubsystem subsystem) {
    return new FlywheelVoltageCommand(subsystem.claw, () -> -0.5).withTimeout(0.2);
  }
}
