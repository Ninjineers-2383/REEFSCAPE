package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.QuarrelPresets.QuarrelPosition;
import frc.robot.commands.flywheel.FlywheelVoltageCommand;
import frc.robot.commands.position_joint.PositionJointPositionCommand;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class QuarrelCommands {
  private QuarrelCommands() {}

  public static Command QuarrelCommand(
      PositionJoint elevator,
      PositionJoint pivot,
      DoubleSupplier elevatorPositionMeters,
      Supplier<Rotation2d> pivotRotation) {
    return new ParallelCommandGroup(
        new PositionJointPositionCommand(elevator, elevatorPositionMeters),
        new PositionJointPositionCommand(pivot, () -> pivotRotation.get().getRotations()));
  }

  public static Command QuarrelCommand(
      PositionJoint elevator, PositionJoint pivot, Supplier<QuarrelPosition> position) {
    return new ParallelCommandGroup(
        new PositionJointPositionCommand(elevator, () -> position.get().elevatorPositionMeters()),
        new PositionJointPositionCommand(
            pivot, () -> position.get().pivotRotation().getRotations()));
  }

  public static Command ScoreCommand(PositionJoint elevator, PositionJoint pivot, Flywheel claw) {
    // Elevator goes down by a specified amount and feeder outtakes
    return new SequentialCommandGroup(
        new PositionJointPositionCommand(
            elevator, () -> elevator.getPosition() - QuarrelPresets.getScore()),
        new FlywheelVoltageCommand(claw, () -> 5.0).withTimeout(1.0),
        new FlywheelVoltageCommand(claw, () -> 0.0).withTimeout(0.1));
  }
}
