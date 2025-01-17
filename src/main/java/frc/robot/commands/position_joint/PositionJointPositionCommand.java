package frc.robot.commands.position_joint;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.position_joint.PositionJoint;
import java.util.function.DoubleSupplier;

public class PositionJointPositionCommand extends Command {
  private final PositionJoint positionJoint;
  private final DoubleSupplier position;

  public PositionJointPositionCommand(PositionJoint positionJoint, DoubleSupplier position) {
    this.positionJoint = positionJoint;
    this.position = position;

    addRequirements(positionJoint);
  }

  @Override
  public void initialize() {
    positionJoint.setPosition(position.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return positionJoint.isFinished();
  }
}
