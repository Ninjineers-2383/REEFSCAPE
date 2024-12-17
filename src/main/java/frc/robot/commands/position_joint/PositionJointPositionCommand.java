package frc.robot.commands.position_joint;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.position_joint.PositionJoint;

public class PositionJointPositionCommand extends Command {
  private final PositionJoint positionJoint;
  private final double position;

  public PositionJointPositionCommand(PositionJoint positionJoint, double position) {
    this.positionJoint = positionJoint;
    this.position = position;

    addRequirements(positionJoint);
  }

  @Override
  public void initialize() {
    positionJoint.setPosition(position);
  }

  @Override
  public boolean isFinished() {
    return positionJoint.isFinished();
  }
}
