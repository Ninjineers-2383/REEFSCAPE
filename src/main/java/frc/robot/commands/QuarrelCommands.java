package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
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
import java.util.Set;
import java.util.function.Supplier;

public class QuarrelCommands {
  private QuarrelCommands() {}

  public static record QuarrelSubsystem(
      PositionJoint elevator,
      PositionJoint pivot,
      Flywheel claw,
      PositionJoint intakePivot,
      Flywheel groundIntake,
      DigitalSensor bottomBeamBreak,
      DigitalSensor intakeBeamBreak) {}

  //   Commands.sequence(
  // Commands.waitUntil(() -> elevator.getPosition() > 0.6),
  // new PositionJointPositionCommand(
  //     pivot, () -> QuarrelPresets.getL4().pivotRotation().getRotations())),
  public static Command PresetCommand(
      QuarrelSubsystem subsystem, Supplier<QuarrelPosition> position) {
    return Commands.parallel(
        Commands.sequence(
            Commands.either(
                // Commands.waitUntil(() -> subsystem.elevator.getPosition() > 0.6),
                Commands.none(),
                Commands.none(),
                () -> position.get().pivotRotation().getDegrees() > 98),
            new PositionJointPositionCommand(
                subsystem.pivot, () -> position.get().pivotRotation().getRotations()),
            new DeferredCommand(
                () ->
                    Commands.waitSeconds(
                        position.get().pivotRotation().getRotations() > 0.5 ? 0.5 : 0.0),
                Set.of())),
        new PositionJointPositionCommand(
            subsystem.elevator, () -> position.get().elevatorPositionMeters()));
  }

  public static Command ScoreCommand(QuarrelSubsystem subsystem) {
    // Score by running claw at full speed until beam breaks are no longer triggered
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new WaitUntilCommand(subsystem.bottomBeamBreak.getTrigger().negate()),
            new FlywheelVoltageCommand(
                subsystem.claw,
                () ->
                    (subsystem.pivot.getPosition() > 0.5 || subsystem.pivot.getPosition() < 0.0)
                        ? -10
                        : 10.0)),
        new WaitCommand(0.1),
        new FlywheelVoltageCommand(subsystem.claw, () -> 0.0).withTimeout(0.02));
  }

  public static Command TransferCommand(QuarrelSubsystem subsystem) {
    Command tranfer1 =
        Commands.deadline(
            Commands.sequence(
                new PrintCommand("Transfer Command Started"),
                new WaitUntilCommand(() -> subsystem.intakePivot.getPosition() < 0.1),
                new PositionJointPositionCommand(
                    subsystem.pivot,
                    () -> QuarrelPresets.getTransferDown().pivotRotation().getRotations()),
                new PrintCommand("Transfer Command Pivot Finished"),
                new PositionJointPositionCommand(
                    subsystem.elevator,
                    () -> QuarrelPresets.getTransferDown().elevatorPositionMeters()),
                new PrintCommand("Transfer Command Move Finished")),
            Commands.sequence(
                new FlywheelVoltageCommand(subsystem.groundIntake, () -> 0.0).withTimeout(0.2),
                new PositionJointPositionCommand(subsystem.intakePivot, () -> -1),
                new FlywheelVoltageCommand(subsystem.groundIntake, () -> 12.0).withTimeout(0.2),
                Commands.waitUntil(subsystem.intakeBeamBreak().getTrigger()),
                new FlywheelVoltageCommand(subsystem.groundIntake, () -> 0.0).withTimeout(0.2)));

    Command transfer =
        new SequentialCommandGroup(
            Commands.parallel(
                new FlywheelVoltageCommand(subsystem.claw, () -> -3.0).withTimeout(0.2),
                new FlywheelVoltageCommand(subsystem.groundIntake, () -> 12.0).withTimeout(0.2)),
            new WaitUntilCommand(subsystem.bottomBeamBreak.getTrigger()),
            new WaitCommand(0.15),
            Commands.parallel(
                new FlywheelVoltageCommand(subsystem.claw, () -> 0.0).withTimeout(0.2),
                new FlywheelVoltageCommand(subsystem.groundIntake, () -> 0.0).withTimeout(0.2)),
            new PrintCommand("Transfer Command Index Finished"));

    return Commands.sequence(
        tranfer1, Commands.either(Commands.none(), transfer, subsystem.bottomBeamBreak.getTrigger())
        // new ForkCommand(new PositionJointPositionCommand(subsystem.intakePivot, () -> 0.22))
        );
  }

  public static Command IntakeAlgaeCommand(QuarrelSubsystem subsystem) {
    return new FlywheelVoltageCommand(subsystem.claw, () -> -3.0).withTimeout(0.2);
  }

  public static Command HoldAlgaeCommand(QuarrelSubsystem subsystem) {
    return new FlywheelVoltageCommand(subsystem.claw, () -> -0.5).withTimeout(0.2);
  }
}
