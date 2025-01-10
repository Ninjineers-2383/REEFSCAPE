package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
      PositionJoint elevator,
      PositionJoint pivot,
      Flywheel claw,
      Supplier<QuarrelPosition> position,
      DoubleSupplier clawVoltage) {
    return new ParallelCommandGroup(
        new PositionJointPositionCommand(elevator, () -> position.get().elevatorPositionMeters()),
        new PositionJointPositionCommand(
            pivot, () -> position.get().pivotRotation().getRotations()),
        new FlywheelVoltageCommand(claw, clawVoltage));
  }

  public static Command ScoreCommand(PositionJoint elevator, PositionJoint pivot, Flywheel claw) {
    // Elevator goes down by a specified amount and feeder outtakes
    return new ParallelCommandGroup(
        new PositionJointPositionCommand(pivot, () -> QuarrelPresets.getScore().getRotations()),
        new SequentialCommandGroup(
            new WaitCommand(0.5),
            new FlywheelVoltageCommand(claw, () -> 5.0)
                .withTimeout(1.0)
                .andThen(new FlywheelVoltageCommand(claw, () -> 0.0))));
  }

  public static Command TransferCommand(
      PositionJoint elevator, PositionJoint pivot, Flywheel claw) {
    return new SequentialCommandGroup(
        new PositionJointPositionCommand(
            elevator, () -> QuarrelPresets.getTransferUp().elevatorPositionMeters()),
        new ParallelCommandGroup(
            new PositionJointPositionCommand(
                pivot, () -> QuarrelPresets.getTransferUp().pivotRotation().getRotations()),
            new FlywheelVoltageCommand(claw, () -> -6.0).withTimeout(0.1)),
        new PositionJointPositionCommand(
            elevator, () -> QuarrelPresets.getTransferDown().elevatorPositionMeters()),
        new PositionJointPositionCommand(
            elevator, () -> QuarrelPresets.getTransferUp().elevatorPositionMeters()),
        new FlywheelVoltageCommand(claw, () -> 0.0).withTimeout(0.1));
  }
}
