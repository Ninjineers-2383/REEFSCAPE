package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
      DigitalSensor topBeamBreak,
      DigitalSensor bottomBeamBreak) {}

  public static Command ElevatorCommand(
      QuarrelSubsystem subsystem, Supplier<QuarrelPosition> position) {
    return new PositionJointPositionCommand(
        subsystem.elevator, () -> position.get().elevatorPositionMeters());
  }

  public static Command FlipCommand(
      QuarrelSubsystem subsystem, Supplier<QuarrelPosition> position) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new WaitUntilCommand(
                subsystem
                    .bottomBeamBreak
                    .getTrigger()
                    .negate()), // Wait until the bottom beam break is untriggered
            new FlywheelVoltageCommand(subsystem.claw, () -> -3.0)), // Back game piece out of claw
        new FlywheelVoltageCommand(subsystem.claw, () -> 0.0).withTimeout(0.1), // Stop the claw
        new ParallelCommandGroup(
            new PositionJointPositionCommand(
                subsystem.pivot,
                () -> position.get().pivotRotation().getRotations()), // Flip the pivot
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    // new WaitUntilCommand(
                    //     subsystem.topBeamBreak
                    //         .getTrigger()), // Wait until the top beam break is untriggered
                    new WaitCommand(0.5),
                    new FlywheelVoltageCommand(subsystem.claw, () -> 1.0)), // Intake the game piece
                new FlywheelVoltageCommand(subsystem.claw, () -> 0.0)
                    .withTimeout(0.02))) // Stop the claw
        );
  }

  public static Command ScoreCommand(QuarrelSubsystem subsystem) {
    // Score by running claw at full speed until beam breaks are no longer triggered
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new WaitUntilCommand(
                subsystem
                    .topBeamBreak
                    .getTrigger()
                    .or(subsystem.bottomBeamBreak.getTrigger())
                    .negate()),
            new FlywheelVoltageCommand(
                subsystem.claw, () -> subsystem.pivot.getPosition() > 0.5 ? -7.0 : 7.0)),
        new WaitCommand(1),
        new FlywheelVoltageCommand(subsystem.claw, () -> 0.0).withTimeout(0.02));
  }

  public static Command TransferCommand(QuarrelSubsystem subsystem) {
    return new SequentialCommandGroup(
        new PrintCommand("Transfer Command Started"),
        new PositionJointPositionCommand(
            subsystem.pivot, () -> QuarrelPresets.getTransferDown().pivotRotation().getRotations()),
        new PrintCommand("Transfer Command Pivot Finished"),
        new PositionJointPositionCommand(
            subsystem.elevator, () -> QuarrelPresets.getTransferDown().elevatorPositionMeters()),
        new PrintCommand("Transfer Command Move Finished"),
        new FlywheelVoltageCommand(subsystem.claw, () -> 5.0).withTimeout(0.2),
        new WaitUntilCommand(subsystem.bottomBeamBreak.getTrigger()),
        new FlywheelVoltageCommand(subsystem.claw, () -> 0.0).withTimeout(0.2),
        new PrintCommand("Transfer Command Index Finished"));
  }
}
