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
  PositionJoint pivot;

  LoggedTunableNumber elevatorPosition = new LoggedTunableNumber("Components/ElevatorPosition", 0);
  LoggedTunableNumber pivotPosition = new LoggedTunableNumber("Components/PivotPosition", 0);

  public Components(PositionJoint elevator, PositionJoint pivot) {
    this.elevator = elevator;
    this.pivot = pivot;
  }

  @Override
  public void periodic() {
    Pose3d[] poses = new Pose3d[1];
    poses[0] =
        new Pose3d(
            new Translation3d(0, 0, 0.334 + elevator.getPosition()),
            new Rotation3d(pivot.getPosition(), 0, 0));
    // poses[0] =
    //     new Pose3d(
    //         new Translation3d(0, 0, 0.334 + elevatorPosition.get()),
    //         new Rotation3d(Math.toRadians(pivotPosition.get()), 0, 0));

    Logger.recordOutput("Components", poses);
  }
}
