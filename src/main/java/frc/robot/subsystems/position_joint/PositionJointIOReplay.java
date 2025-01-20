package frc.robot.subsystems.position_joint;

public class PositionJointIOReplay implements PositionJointIO {
  private final String name;

  public PositionJointIOReplay(String name) {
    this.name = name;
  }

  @Override
  public String getName() {
    return name;
  }
}
