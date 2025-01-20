package frc.robot.subsystems.piece_detection;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PieceDetection extends SubsystemBase {
  private PieceDetectionIO pieceDetection;
  private PieceDetectionIOInputsAutoLogged inputs = new PieceDetectionIOInputsAutoLogged();

  private final String name;
  private final Supplier<Pose3d> robotPose;

  private Pose3d gamePiecePose = new Pose3d();

  public PieceDetection(PieceDetectionIO io, Supplier<Pose3d> robotPose) {
    pieceDetection = io;

    name = pieceDetection.getName();
    this.robotPose = robotPose;
  }

  @Override
  public void periodic() {
    pieceDetection.updateInputs(inputs);

    if (inputs.seesTarget) {
      gamePiecePose = robotPose.get().transformBy(inputs.robotToPieceTransform);

      Logger.recordOutput(name + "/Piece Pose", gamePiecePose);
    }
    Logger.processInputs(name, inputs);
  }

  public double getPitch() {
    return inputs.pitch;
  }

  public double getYaw() {
    return inputs.yaw;
  }

  public double getArea() {
    return inputs.area;
  }

  public boolean pieceDetected() {
    return inputs.seesTarget;
  }

  public Pose3d getPiecePose() {
    return gamePiecePose;
  }
}
