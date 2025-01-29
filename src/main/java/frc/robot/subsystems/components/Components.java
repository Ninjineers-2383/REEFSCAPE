package frc.robot.subsystems.components;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Components extends SubsystemBase {
  PositionJoint elevator;

  LoggedTunableNumber elevatorPosition = new LoggedTunableNumber("Components/ElevatorPosition", 0);

  public Components(PositionJoint elevator) {
    this.elevator = elevator;
  }

  @Override
  public void periodic() {
    Pose3d[] poses = new Pose3d[2];
    poses[0] =
        new Pose3d(new Translation3d(0, 0, elevator.getPosition() / 2.0), new Rotation3d(0, 0, 0));
    poses[1] = new Pose3d(new Translation3d(0, 0, elevator.getPosition()), new Rotation3d(0, 0, 0));
    Logger.recordOutput("Components", poses);
  }
}
